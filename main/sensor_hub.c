/**
 * @file sensor_hub.c
 * @brief Implementacao do agregador de sensores (ver sensor_hub.h).
 *
 * Desvio MISRA (Rule 21.6 / Dir 4.9): o JSON de telemetria e montado com snprintf
 * via a macro de append delimitada APPEND. E a forma segura (com limite de
 * tamanho) e legivel de montar o texto; nao faz parte da cadeia de PWM.
 */
#include "sensor_hub.h"

#include <stdio.h>
#include <string.h>
#include "config.h"
#include "i2c_bus.h"
#include "app_time.h"
#include "esp_log.h"
#include "freertos/task.h"

/* A IMU e lida a ~200 Hz (5 ms), bem acima dos 50 Hz da malha de controle, para
 * que cada ciclo de controle disponha sempre de uma amostra fresca. Com a IMU e o
 * controle na mesma taxa (antes, 20 ms ambos) e tarefas nao sincronizadas, a fase
 * derivava e o controle ora reusava a mesma amostra (ciclo pulado), ora media dt
 * irregular, degradando a rejeicao de perturbacao no hover. A fusao de atitude usa
 * dt real por amostra, entao amostrar mais rapido apenas melhora a precisao; o
 * magnetometro se auto-limita pelo bit DRDY. */
#define MPU_READ_INTERVAL_MS      (5U)
#define BMP_READ_INTERVAL_MS      (200U)
#define SENSOR_RETRY_INTERVAL_MS  (5000U)
#define SEA_LEVEL_HPA             (1013.25f)

static const char *TAG = "sensor-hub";

/** @brief Converte um booleano no literal JSON correspondente. */
static const char *bool_str(bool value)
{
    return value ? "true" : "false";
}

/** @brief Idade (ms) de um snapshot a partir do seu timestamp (0 se nunca lido). */
static uint32_t age_ms(uint32_t updated_at_ms)
{
    return (updated_at_ms != 0U) ? (millis() - updated_at_ms) : 0U;
}

/**
 * @brief Tenta reinicializar, periodicamente, os sensores I2C que estao offline.
 *
 * Permite recuperar de uma falha transitoria (fio mal contato, reset do sensor)
 * sem reiniciar o firmware, sem martelar o barramento a cada ciclo.
 */
static void retry_offline_sensors(sensor_hub_t *self, uint32_t now)
{
    if ((now - self->last_retry_ms) < SENSOR_RETRY_INTERVAL_MS)
    {
        return;
    }
    self->last_retry_ms = now;
    if (!mpu9259_is_online(&self->mpu))
    {
        (void)mpu9259_begin(&self->mpu);
    }
    if (!bmp280_is_online(&self->bmp))
    {
        (void)bmp280_begin(&self->bmp);
    }
}

void sensor_hub_init(sensor_hub_t *self)
{
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    (void)memset(self, 0, sizeof(*self));
    self->data_mutex = mux;
    self->last_mpu_read_ms = 0U;
    self->last_bmp_read_ms = 0U;
    self->last_retry_ms = 0U;
}

void sensor_hub_begin(sensor_hub_t *self, int sda_gpio, int scl_gpio,
                      int gps_uart_port, int gps_rx_gpio, int gps_tx_gpio)
{
    mpu9259_data_t initial_mpu;
    const esp_err_t i2c_err = i2c_bus_init(sda_gpio, scl_gpio, (uint32_t)I2C_CLOCK_HZ);
    if (i2c_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar I2C (%s)", esp_err_to_name(i2c_err));
    }

    neo6m_begin(&self->gps, gps_uart_port, GPS_BAUD, gps_rx_gpio, gps_tx_gpio);

    ESP_LOGI(TAG, "MPU925x: %s", mpu9259_begin(&self->mpu) ? "conectado" : "nao encontrado");
    (void)memset(&initial_mpu, 0, sizeof(initial_mpu));
    if (mpu9259_is_online(&self->mpu) && mpu9259_read(&self->mpu, &initial_mpu))
    {
        self->mpu_data = initial_mpu;
        ESP_LOGI(TAG, "Magnetometro: %s",
                 initial_mpu.magnetometer_available ? "conectado" : "nao encontrado");
    }
    else
    {
        ESP_LOGI(TAG, "Magnetometro: nao encontrado");
    }

    ESP_LOGI(TAG, "BMP280: %s", bmp280_begin(&self->bmp) ? "conectado" : "nao encontrado");
    ESP_LOGI(TAG, "GPS NEO-6M: porta serial inicializada");
}

void sensor_hub_update(sensor_hub_t *self)
{
    const uint32_t now = millis();
    neo6m_data_t next_gps;
    mpu9259_data_t next_mpu = self->mpu_data;
    bmp280_data_t next_bmp = self->bmp_data;
    bool has_mpu_update = false;
    bool has_bmp_update = false;

    neo6m_update(&self->gps);
    next_gps = neo6m_data(&self->gps);

    if (mpu9259_is_online(&self->mpu) && ((now - self->last_mpu_read_ms) >= MPU_READ_INTERVAL_MS))
    {
        self->last_mpu_read_ms = now;
        (void)mpu9259_read(&self->mpu, &next_mpu);
        has_mpu_update = true;
    }

    if (bmp280_is_online(&self->bmp) && ((now - self->last_bmp_read_ms) >= BMP_READ_INTERVAL_MS))
    {
        self->last_bmp_read_ms = now;
        (void)bmp280_read(&self->bmp, &next_bmp, SEA_LEVEL_HPA);
        has_bmp_update = true;
    }

    taskENTER_CRITICAL(&self->data_mutex);
    self->gps_data = next_gps;
    if (has_mpu_update)
    {
        self->mpu_data = next_mpu;
    }
    if (has_bmp_update)
    {
        self->bmp_data = next_bmp;
    }
    taskEXIT_CRITICAL(&self->data_mutex);

    retry_offline_sensors(self, now);
}

bool sensor_hub_get_mpu_snapshot(sensor_hub_t *self, mpu9259_data_t *data)
{
    taskENTER_CRITICAL(&self->data_mutex);
    *data = self->mpu_data;
    taskEXIT_CRITICAL(&self->data_mutex);
    return data->valid;
}

bool sensor_hub_get_bmp_snapshot(sensor_hub_t *self, bmp280_data_t *data)
{
    taskENTER_CRITICAL(&self->data_mutex);
    *data = self->bmp_data;
    taskEXIT_CRITICAL(&self->data_mutex);
    return data->valid;
}

bool sensor_hub_get_gps_snapshot(sensor_hub_t *self, neo6m_data_t *data)
{
    taskENTER_CRITICAL(&self->data_mutex);
    *data = self->gps_data;
    taskEXIT_CRITICAL(&self->data_mutex);
    return data->valid;
}

size_t sensor_hub_to_json(sensor_hub_t *self, char *out, size_t cap)
{
    mpu9259_data_t mpu;
    bmp280_data_t bmp;
    neo6m_data_t gps;
    const bool mpu_online = mpu9259_is_online(&self->mpu);
    const bool bmp_online = bmp280_is_online(&self->bmp);
    size_t offset = 0U;

    if ((out == NULL) || (cap == 0U))
    {
        return 0U;
    }

    taskENTER_CRITICAL(&self->data_mutex);
    mpu = self->mpu_data;
    bmp = self->bmp_data;
    gps = self->gps_data;
    taskEXIT_CRITICAL(&self->data_mutex);

/* Append delimitado: nunca escreve alem de 'cap'. */
#define APPEND(...)                                                       \
    do {                                                                  \
        if (offset < cap) {                                               \
            const int written = snprintf(&out[offset], cap - offset, __VA_ARGS__); \
            if (written > 0) {                                            \
                offset += (size_t)written;                               \
                if (offset > cap) { offset = cap; }                      \
            }                                                             \
        }                                                                 \
    } while (0)

    APPEND("{\"mpu9259\":{\"online\":%s,\"valid\":%s,\"magnetometerAvailable\":%s",
           bool_str(mpu_online), bool_str(mpu.valid), bool_str(mpu.magnetometer_available));
    APPEND(",\"accelG\":[%.4f,%.4f,%.4f]", mpu.accel_x, mpu.accel_y, mpu.accel_z);
    APPEND(",\"gyroDps\":[%.3f,%.3f,%.3f]", mpu.gyro_x, mpu.gyro_y, mpu.gyro_z);
    APPEND(",\"magUt\":[%.2f,%.2f,%.2f]", mpu.mag_x, mpu.mag_y, mpu.mag_z);
    APPEND(",\"attitudeDeg\":[%.2f,%.2f]", mpu.roll_deg, mpu.pitch_deg);
    APPEND(",\"realAttitude\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
           mpu.roll_deg, mpu.pitch_deg, mpu.yaw_deg);
    APPEND(",\"accelNormG\":%.3f", mpu.accel_norm_g);
    APPEND(",\"accelCorrectionUsed\":%s", bool_str(mpu.accel_correction_used));
    APPEND(",\"accelSampleRejected\":%s", bool_str(mpu.accel_sample_rejected));
    APPEND(",\"calibrationComplete\":%s", bool_str(mpu.calibration_complete));
    APPEND(",\"calibrationProgress\":%u", (unsigned)mpu.calibration_progress);
    APPEND(",\"rejectedSamples\":%u", (unsigned)mpu.rejected_sample_count);
    APPEND(",\"temperatureC\":%.2f", mpu.temperature_c);
    APPEND(",\"ageMs\":%u", (unsigned)age_ms(mpu.updated_at_ms));
    APPEND(",\"errors\":%u", (unsigned)mpu.error_count);

    APPEND("},\"bmp280\":{\"online\":%s,\"valid\":%s", bool_str(bmp_online), bool_str(bmp.valid));
    APPEND(",\"temperatureC\":%.2f", bmp.temperature_c);
    APPEND(",\"pressureHpa\":%.2f", bmp.pressure_hpa);
    APPEND(",\"pressureReferenceHpa\":%.2f", bmp.pressure_reference_hpa);
    APPEND(",\"altitudeAbsoluteM\":%.2f", bmp.altitude_absolute_m);
    APPEND(",\"altitudeM\":%.2f", bmp.altitude_m);
    APPEND(",\"altitudeReferenceM\":%.2f", bmp.altitude_reference_m);
    APPEND(",\"referenceReady\":%s", bool_str(bmp.reference_ready));
    APPEND(",\"referenceProgress\":%u", (unsigned)bmp.reference_progress);
    APPEND(",\"altitudeSampleRejected\":%s", bool_str(bmp.altitude_sample_rejected));
    APPEND(",\"rejectedSamples\":%u", (unsigned)bmp.rejected_sample_count);
    APPEND(",\"ageMs\":%u", (unsigned)age_ms(bmp.updated_at_ms));
    APPEND(",\"errors\":%u", (unsigned)bmp.error_count);

    APPEND("},\"gps\":{\"valid\":%s,\"signalLost\":%s", bool_str(gps.valid), bool_str(gps.signal_lost));
    APPEND(",\"latitude\":%.6f", gps.latitude);
    APPEND(",\"longitude\":%.6f", gps.longitude);
    APPEND(",\"altitudeM\":%.2f", (double)gps.altitude_m);
    APPEND(",\"speedKmh\":%.2f", (double)gps.speed_kmh);
    APPEND(",\"satellites\":%u", (unsigned)gps.satellites);
    APPEND(",\"fixQuality\":%u", (unsigned)gps.fix_quality);
    APPEND(",\"hdop\":%.2f", (double)gps.hdop);
    APPEND(",\"ageMs\":%u", (unsigned)age_ms(gps.updated_at_ms));
    APPEND(",\"checksumErrors\":%u", (unsigned)gps.checksum_errors);
    APPEND("}}");

#undef APPEND

    return offset;
}
