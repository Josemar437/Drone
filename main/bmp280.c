/**
 * @file bmp280.c
 * @brief Implementacao do driver do barometro (ver bmp280.h).
 *
 * A compensacao de pressao/temperatura usa aritmetica inteira de 64 bits e a
 * filtragem de altitude opera em ponto flutuante. Registradores e algoritmo:
 * datasheet do BMP280.
 */
#include "bmp280.h"

#include <math.h>
#include "i2c_bus.h"
#include "app_time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BMP_ADDRESS_LOW             (0x76U)
#define BMP_ADDRESS_HIGH            (0x77U)
#define BMP_CHIP_ID                 (0xD0U)
#define BMP_RESET                   (0xE0U)
#define BMP_CALIBRATION_START       (0x88U)
#define BMP_CTRL_MEAS               (0xF4U)
#define BMP_CONFIG                  (0xF5U)
#define BMP_PRESS_MSB               (0xF7U)
#define STABILIZATION_SAMPLE_TARGET (15U)
#define REFERENCE_SAMPLE_TARGET     (40U)
#define ALTITUDE_FILTER_ALPHA       (0.12f)
#define MAX_ALTITUDE_STEP_M         (1.20f)
#define MAX_REJECTED_SAMPLE_COUNT   (65535U)

static const uint8_t BMP_ADDRESSES[2] = { BMP_ADDRESS_LOW, BMP_ADDRESS_HIGH };

/** @brief Le 2 bytes little-endian sem sinal (formato dos coeficientes do BMP). */
static uint16_t decode_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
}

/** @brief Idem, interpretado como inteiro com sinal. */
static int16_t decode_s16_le(const uint8_t *p)
{
    return (int16_t)decode_u16_le(p);
}

/** @brief Atalho de leitura I2C no endereco do barometro. */
static bool read_registers(const bmp280_t *self, uint8_t reg, uint8_t *buffer, size_t length)
{
    return i2c_bus_read_regs(self->address, reg, buffer, length);
}

/** @brief Atalho de escrita I2C no endereco do barometro. */
static bool write_register(const bmp280_t *self, uint8_t reg, uint8_t value)
{
    return i2c_bus_write_reg(self->address, reg, value);
}

/**
 * @brief Le os coeficientes de calibracao de fabrica (dig_T e dig_P).
 *
 * Sao indispensaveis para a compensacao: sem eles a pressao/temperatura nao tem
 * significado fisico. Gravados de fabrica na NVM do sensor.
 *
 * @return true se os coeficientes foram lidos e parecem validos (nao-zero).
 */
static bool read_calibration(bmp280_t *self)
{
    uint8_t raw[24];
    if (!read_registers(self, BMP_CALIBRATION_START, raw, sizeof(raw)))
    {
        return false;
    }
    self->dig_t1 = decode_u16_le(&raw[0]);
    self->dig_t2 = decode_s16_le(&raw[2]);
    self->dig_t3 = decode_s16_le(&raw[4]);
    self->dig_p1 = decode_u16_le(&raw[6]);
    self->dig_p2 = decode_s16_le(&raw[8]);
    self->dig_p3 = decode_s16_le(&raw[10]);
    self->dig_p4 = decode_s16_le(&raw[12]);
    self->dig_p5 = decode_s16_le(&raw[14]);
    self->dig_p6 = decode_s16_le(&raw[16]);
    self->dig_p7 = decode_s16_le(&raw[18]);
    self->dig_p8 = decode_s16_le(&raw[20]);
    self->dig_p9 = decode_s16_le(&raw[22]);
    return (self->dig_t1 != 0U) && (self->dig_p1 != 0U);
}

/**
 * @brief Reinicia o aprendizado da referencia de altitude e o filtro.
 *
 * Apos isto, o driver volta a coletar amostras de estabilizacao e a media de
 * pressao que definem o "zero" de altitude.
 */
static void reset_altitude_reference(bmp280_t *self)
{
    self->pressure_reference_sum_hpa = 0.0f;
    self->pressure_reference_hpa = 0.0f;
    self->altitude_reference_m = 0.0f;
    self->filtered_relative_altitude_m = 0.0f;
    self->previous_raw_relative_altitude_m = 0.0f;
    self->stabilization_sample_count = 0U;
    self->reference_sample_count = 0U;
    self->rejected_sample_count = 0U;
    self->altitude_filter_initialized = false;
    self->reference_ready = false;
}

/**
 * @brief Converte pressao em altitude pela formula barometrica internacional.
 *
 * @param pressure_hpa  Pressao medida (hPa).
 * @param sea_level_hpa Pressao de referencia (hPa).
 * @return Altitude em metros relativa a essa referencia.
 */
static float calculate_altitude(float pressure_hpa, float sea_level_hpa)
{
    return 44330.0f * (1.0f - powf(pressure_hpa / sea_level_hpa, 0.19029495f));
}

/**
 * @brief Aprende a referencia de altitude e mantem a altura relativa filtrada.
 *
 * Fases: (1) descarta as primeiras amostras ate o sensor estabilizar; (2) faz a
 * media da pressao para fixar o "zero"; (3) reporta a altura relativa, aplicando
 * filtro passa-baixa e descartando saltos maiores que o passo maximo (ruido).
 * O campo reference_progress reflete o andamento das fases 1-2.
 */
static void update_relative_altitude(bmp280_t *self, bmp280_data_t *data, float sea_level_hpa)
{
    float raw_relative_altitude_m;
    bool spike_detected;

    data->altitude_sample_rejected = false;
    data->reference_ready = self->reference_ready;
    data->reference_progress = 0U;
    data->pressure_reference_hpa = self->pressure_reference_hpa;
    data->altitude_reference_m = self->altitude_reference_m;
    data->altitude_m = 0.0f;
    data->rejected_sample_count = self->rejected_sample_count;

    if (self->stabilization_sample_count < STABILIZATION_SAMPLE_TARGET)
    {
        self->stabilization_sample_count++;
        data->reference_progress = (uint8_t)((self->stabilization_sample_count * 100U) /
            (STABILIZATION_SAMPLE_TARGET + REFERENCE_SAMPLE_TARGET));
        return;
    }

    if (!self->reference_ready)
    {
        self->pressure_reference_sum_hpa += data->pressure_hpa;
        self->reference_sample_count++;
        data->reference_progress = (uint8_t)(((STABILIZATION_SAMPLE_TARGET + self->reference_sample_count) * 100U) /
            (STABILIZATION_SAMPLE_TARGET + REFERENCE_SAMPLE_TARGET));
        if (self->reference_sample_count < REFERENCE_SAMPLE_TARGET)
        {
            return;
        }

        self->pressure_reference_hpa = self->pressure_reference_sum_hpa / (float)REFERENCE_SAMPLE_TARGET;
        self->altitude_reference_m = calculate_altitude(self->pressure_reference_hpa, sea_level_hpa);
        self->previous_raw_relative_altitude_m = data->altitude_absolute_m - self->altitude_reference_m;
        self->filtered_relative_altitude_m = 0.0f;
        self->altitude_filter_initialized = true;
        self->reference_ready = true;
    }

    raw_relative_altitude_m = data->altitude_absolute_m - self->altitude_reference_m;
    spike_detected = self->altitude_filter_initialized &&
        (fabsf(raw_relative_altitude_m - self->previous_raw_relative_altitude_m) > MAX_ALTITUDE_STEP_M);
    if (spike_detected)
    {
        data->altitude_sample_rejected = true;
        if (self->rejected_sample_count < MAX_REJECTED_SAMPLE_COUNT)
        {
            self->rejected_sample_count++;
        }
    }
    else
    {
        self->previous_raw_relative_altitude_m = raw_relative_altitude_m;
        self->filtered_relative_altitude_m +=
            ALTITUDE_FILTER_ALPHA * (raw_relative_altitude_m - self->filtered_relative_altitude_m);
    }

    data->reference_ready = self->reference_ready;
    data->reference_progress = 100U;
    data->pressure_reference_hpa = self->pressure_reference_hpa;
    data->altitude_reference_m = self->altitude_reference_m;
    data->altitude_m = self->filtered_relative_altitude_m;
    data->rejected_sample_count = self->rejected_sample_count;
}

bool bmp280_begin(bmp280_t *self)
{
    uint8_t identity = 0U;
    size_t index;

    self->error_count = 0U;
    self->fine_temperature = 0;
    self->online = false;

    for (index = 0U; index < (sizeof(BMP_ADDRESSES) / sizeof(BMP_ADDRESSES[0])); index++)
    {
        self->address = BMP_ADDRESSES[index];
        if (read_registers(self, BMP_CHIP_ID, &identity, 1U) && (identity == 0x58U))
        {
            self->online = true;
            break;
        }
    }
    if (!self->online)
    {
        return false;
    }

    (void)write_register(self, BMP_RESET, 0xB6U);
    vTaskDelay(pdMS_TO_TICKS(10));
    self->online = read_calibration(self);
    self->online = self->online && write_register(self, BMP_CONFIG, 0x10U);
    self->online = self->online && write_register(self, BMP_CTRL_MEAS, 0x57U);
    reset_altitude_reference(self);
    self->error_count = 0U;
    return self->online;
}

bool bmp280_read(bmp280_t *self, bmp280_data_t *data, float sea_level_hpa)
{
    uint8_t raw[6];
    int32_t adc_pressure;
    int32_t adc_temperature;
    int32_t var1;
    int32_t var2;
    int64_t p_var1;
    int64_t p_var2;
    int64_t pressure;

    data->valid = false;
    if (!self->online || !read_registers(self, BMP_PRESS_MSB, raw, sizeof(raw)))
    {
        self->error_count++;
        data->error_count = self->error_count;
        return false;
    }

    adc_pressure = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);
    adc_temperature = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);
    if ((adc_pressure == 0x80000) || (adc_temperature == 0x80000) || (self->dig_p1 == 0U))
    {
        self->error_count++;
        data->error_count = self->error_count;
        return false;
    }

    var1 = (((adc_temperature >> 3) - ((int32_t)self->dig_t1 << 1)) * (int32_t)self->dig_t2) >> 11;
    var2 = ((((adc_temperature >> 4) - (int32_t)self->dig_t1) *
             ((adc_temperature >> 4) - (int32_t)self->dig_t1)) >> 12);
    var2 = (var2 * (int32_t)self->dig_t3) >> 14;
    self->fine_temperature = var1 + var2;
    data->temperature_c = (float)(((self->fine_temperature * 5) + 128) >> 8) / 100.0f;

    p_var1 = (int64_t)self->fine_temperature - 128000;
    p_var2 = p_var1 * p_var1 * (int64_t)self->dig_p6;
    p_var2 += (p_var1 * (int64_t)self->dig_p5) << 17;
    p_var2 += (int64_t)self->dig_p4 << 35;
    p_var1 = ((p_var1 * p_var1 * (int64_t)self->dig_p3) >> 8) + ((p_var1 * (int64_t)self->dig_p2) << 12);
    p_var1 = ((((int64_t)1 << 47) + p_var1) * (int64_t)self->dig_p1) >> 33;
    if (p_var1 == 0)
    {
        return false;
    }

    pressure = 1048576 - adc_pressure;
    pressure = (((pressure << 31) - p_var2) * 3125) / p_var1;
    p_var1 = ((int64_t)self->dig_p9 * (pressure >> 13) * (pressure >> 13)) >> 25;
    p_var2 = ((int64_t)self->dig_p8 * pressure) >> 19;
    pressure = ((pressure + p_var1 + p_var2) >> 8) + ((int64_t)self->dig_p7 << 4);
    data->pressure_hpa = ((float)pressure / 256.0f) / 100.0f;

    if (!isfinite(data->temperature_c) || !isfinite(data->pressure_hpa) ||
        (data->pressure_hpa < 300.0f) || (data->pressure_hpa > 1200.0f) || (sea_level_hpa <= 0.0f))
    {
        self->error_count++;
        data->error_count = self->error_count;
        return false;
    }

    data->altitude_absolute_m = calculate_altitude(data->pressure_hpa, sea_level_hpa);
    update_relative_altitude(self, data, sea_level_hpa);
    self->error_count = 0U;
    data->error_count = 0U;
    data->updated_at_ms = millis();
    data->valid = true;
    return true;
}

bool bmp280_is_online(const bmp280_t *self)
{
    return self->online;
}
