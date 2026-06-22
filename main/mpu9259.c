/**
 * @file mpu9259.c
 * @brief Implementacao do driver da IMU (ver mpu9259.h).
 *
 * A matematica de fusao, filtragem e calibracao usa uma ordem fixa de operacoes
 * em ponto flutuante. Os registradores e constantes abaixo vem do datasheet do
 * MPU-925x/AK8963.
 */
#include "mpu9259.h"

#include <math.h>
#include "config.h"
#include "i2c_bus.h"
#include "app_time.h"
#include "app_math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ===== Registradores / constantes ===== */
#define MPU_ADDRESS_LOW    (0x68U)
#define MPU_ADDRESS_HIGH   (0x69U)
#define AK8963_ADDRESS     (0x0CU)
#define WHO_AM_I           (0x75U)
#define PWR_MGMT_1         (0x6BU)
#define REG_CONFIG         (0x1AU)
#define GYRO_CONFIG        (0x1BU)
#define ACCEL_CONFIG       (0x1CU)
#define ACCEL_CONFIG_2     (0x1DU)
#define INT_PIN_CFG        (0x37U)
#define ACCEL_XOUT_H       (0x3BU)
#define AK8963_WIA         (0x00U)
#define AK8963_ST1         (0x02U)
#define AK8963_CNTL1       (0x0AU)
#define AK8963_ASAX        (0x10U)
#define GYRO_DLPF_20_HZ    (0x04U)
#define ACCEL_DLPF_20_HZ   (0x04U)

#define ACCEL_LPF_CUTOFF_HZ            (4.0f)
#define MAX_ACCEL_STEP_G               (0.35f)
#define MIN_GRAVITY_G                  (0.85f)
#define MAX_GRAVITY_G                  (1.15f)
#define MOVING_GYRO_WEIGHT             (0.995f)
#define STABLE_GYRO_WEIGHT             (0.98f)
#define MOTION_RATE_THRESHOLD_DPS      (4.0f)
#define GYRO_NOISE_DEADBAND_DPS        (0.65f)
#define CALIBRATION_MAX_GYRO_DPS       (3.0f)
#define CALIBRATION_SAMPLE_TARGET      (150U)
#define MAGNETOMETER_CORRECTION_WEIGHT (0.02f)
#define DEFAULT_DT_SECONDS             (0.02f)
#define MAX_DT_SECONDS                 (0.10f)
#define RADIANS_TO_DEGREES             (57.2957795f)
#define DEGREES_TO_RADIANS             (0.0174532925f)
#define TWO_PI_F                       (6.2831853f)
#define MAX_REJECTED_SAMPLE_COUNT      (65535U)

static const uint8_t MPU_ADDRESSES[2] = { MPU_ADDRESS_LOW, MPU_ADDRESS_HIGH };

/* ===== Helpers locais ===== */

/** @brief Monta um inteiro de 16 bits com sinal a partir de 2 bytes big-endian
 *  (formato do acelerometro/giroscopio da MPU). */
static int16_t decode_big_endian16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[0] << 8U) | (uint16_t)bytes[1]);
}

/** @brief Idem, para 2 bytes little-endian (formato do magnetometro AK8963). */
static int16_t decode_little_endian16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[1] << 8U) | (uint16_t)bytes[0]);
}

/**
 * @brief Coeficiente alpha de um filtro passa-baixa de 1a ordem para o dt dado.
 *
 * Converte a frequencia de corte fixa (ACCEL_LPF_CUTOFF_HZ) no alpha do filtro
 * exponencial, ajustando-se ao intervalo real entre amostras (dt variavel).
 */
static float low_pass_alpha(float dt_seconds)
{
    const float rc_seconds = 1.0f / (TWO_PI_F * ACCEL_LPF_CUTOFF_HZ);
    return dt_seconds / (rc_seconds + dt_seconds);
}

/**
 * @brief Aplica uma zona morta a velocidade angular.
 *
 * Abaixo do limiar, considera-se ruido do giroscopio e retorna 0, evitando que
 * a integracao de ruido cause deriva lenta da atitude com o drone parado.
 */
static float remove_gyro_noise(float rate_dps)
{
    float result = rate_dps;
    if (fabsf(rate_dps) < GYRO_NOISE_DEADBAND_DPS)
    {
        result = 0.0f;
    }
    return result;
}

#if (MPU_REMAP_ROTATE_Y_90 != 0)
/**
 * @brief Rotaciona um vetor +90 graus em torno do eixo Y (novo_x=z, novo_z=-x).
 *
 * Compensa a montagem fisica girada da IMU (ver MPU_REMAP_ROTATE_Y_90 em
 * config.h). Aplicado identicamente a accel e giroscopio para manter o frame
 * consistente.
 */
static void apply_axis_remap(float *x, float *y, float *z)
{
    const float old_x = *x;
    *x = *z;
    *z = -old_x;
    (void)y; /* eixo Y inalterado nesta rotacao */
}
#endif

/** @brief Normaliza um angulo para [-180, 180] graus (yaw continuo). */
static float wrap_angle_180(float angle_deg)
{
    float wrapped = angle_deg;
    while (wrapped > 180.0f)
    {
        wrapped -= 360.0f;
    }
    while (wrapped < -180.0f)
    {
        wrapped += 360.0f;
    }
    return wrapped;
}

/** @brief Atalho de escrita I2C (a MPU e o AK8963 usam enderecos diferentes). */
static bool write_register(uint8_t address, uint8_t reg, uint8_t value)
{
    return i2c_bus_write_reg(address, reg, value);
}

/** @brief Atalho de leitura I2C em bloco. */
static bool read_registers(uint8_t address, uint8_t reg, uint8_t *buffer, size_t length)
{
    return i2c_bus_read_regs(address, reg, buffer, length);
}

/**
 * @brief Zera todo o estado do driver para os valores iniciais.
 *
 * Chamado no inicio de ::mpu9259_begin para que uma reinicializacao parta de um
 * estado conhecido (ganhos de mag = 1, escala de accel = 1, filtros zerados).
 */
static void init_defaults(mpu9259_t *self)
{
    uint8_t axis;
    self->address = MPU_ADDRESS_LOW;
    self->online = false;
    self->magnetometer_available = false;
    for (axis = 0U; axis < 3U; axis++)
    {
        self->mag_adjustment[axis] = 1.0f;
        self->accel_offset[axis] = 0.0f;
        self->gyro_offset[axis] = 0.0f;
        self->accel_calibration_sum[axis] = 0.0f;
        self->gyro_calibration_sum[axis] = 0.0f;
    }
    self->filtered_accel[0] = 0.0f;
    self->filtered_accel[1] = 0.0f;
    self->filtered_accel[2] = 1.0f;
    self->roll_deg = 0.0f;
    self->pitch_deg = 0.0f;
    self->yaw_deg = 0.0f;
    self->accel_scale = 1.0f;
    self->accel_norm_calibration_sum = 0.0f;
    self->calibration_sample_count = 0U;
    self->calibration_complete = false;
    self->magnetometer_yaw_initialized = false;
    self->last_attitude_update_ms = 0U;
    self->sample_seq = 0U;
    self->rejected_sample_count = 0U;
    self->filter_initialized = false;
    self->error_count = 0U;
}

/**
 * @brief Reinicia os filtros e angulos de atitude (mantem a calibracao).
 *
 * Chamado ao concluir a calibracao para que a estimativa de atitude recomece
 * "limpa" com os offsets ja aplicados.
 */
static void reset_filters(mpu9259_t *self)
{
    self->filtered_accel[0] = 0.0f;
    self->filtered_accel[1] = 0.0f;
    self->filtered_accel[2] = 1.0f;
    self->roll_deg = 0.0f;
    self->pitch_deg = 0.0f;
    self->yaw_deg = 0.0f;
    self->magnetometer_yaw_initialized = false;
    self->last_attitude_update_ms = 0U;
    self->sample_seq = 0U;
    self->rejected_sample_count = 0U;
    self->filter_initialized = false;
}

/**
 * @brief Zera os acumuladores e resultados da autocalibracao.
 *
 * Volta offsets a 0 e escala a 1, marcando a calibracao como nao concluida.
 */
static void reset_calibration(mpu9259_t *self)
{
    uint8_t axis;
    for (axis = 0U; axis < 3U; axis++)
    {
        self->accel_offset[axis] = 0.0f;
        self->gyro_offset[axis] = 0.0f;
        self->accel_calibration_sum[axis] = 0.0f;
        self->gyro_calibration_sum[axis] = 0.0f;
    }
    self->calibration_sample_count = 0U;
    self->accel_norm_calibration_sum = 0.0f;
    self->accel_scale = 1.0f;
    self->calibration_complete = false;
}

/**
 * @brief Detecta e configura o magnetometro AK8963 embutido na MPU.
 *
 * Le os fatores de ajuste de sensibilidade de fabrica (ASA) e coloca o chip em
 * medicao continua de 16 bits. Sem ele, o yaw fica so com o giroscopio (deriva).
 *
 * @return true se o AK8963 respondeu e foi configurado; false caso contrario.
 */
static bool initialize_magnetometer(mpu9259_t *self)
{
    uint8_t identity = 0U;
    uint8_t adjustment[3];
    uint8_t i;

    if (!read_registers(AK8963_ADDRESS, AK8963_WIA, &identity, 1U) || (identity != 0x48U))
    {
        return false;
    }

    (void)write_register(AK8963_ADDRESS, AK8963_CNTL1, 0x00U);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (!write_register(AK8963_ADDRESS, AK8963_CNTL1, 0x0FU))
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    if (!read_registers(AK8963_ADDRESS, AK8963_ASAX, adjustment, sizeof(adjustment)))
    {
        return false;
    }
    for (i = 0U; i < 3U; i++)
    {
        self->mag_adjustment[i] = (((float)adjustment[i] - 128.0f) / 256.0f) + 1.0f;
    }

    (void)write_register(AK8963_ADDRESS, AK8963_CNTL1, 0x00U);
    vTaskDelay(pdMS_TO_TICKS(10));
    return write_register(AK8963_ADDRESS, AK8963_CNTL1, 0x16U);
}

/**
 * @brief Acumula amostras em repouso e finaliza a autocalibracao.
 *
 * Enquanto a placa esta estavel (gravidade ~1g e giro baixo), soma as amostras;
 * qualquer movimento reinicia a contagem. Ao atingir o alvo, calcula a escala do
 * acelerometro (corrige o modulo para 1g) e os offsets do giroscopio, e marca a
 * calibracao como concluida. So entao a malha estabilizada pode armar.
 */
static void update_calibration(mpu9259_t *self, float accel_x, float accel_y, float accel_z,
                               float gyro_x, float gyro_y, float gyro_z)
{
    float accel_norm;
    bool stationary;

    if (self->calibration_complete)
    {
        return;
    }

    accel_norm = sqrtf((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));
    stationary = (accel_norm >= MIN_GRAVITY_G) && (accel_norm <= MAX_GRAVITY_G) &&
                 (fabsf(gyro_x) <= CALIBRATION_MAX_GYRO_DPS) &&
                 (fabsf(gyro_y) <= CALIBRATION_MAX_GYRO_DPS) &&
                 (fabsf(gyro_z) <= CALIBRATION_MAX_GYRO_DPS);
    if (!stationary)
    {
        uint8_t axis;
        for (axis = 0U; axis < 3U; axis++)
        {
            self->accel_calibration_sum[axis] = 0.0f;
            self->gyro_calibration_sum[axis] = 0.0f;
        }
        self->calibration_sample_count = 0U;
        self->accel_norm_calibration_sum = 0.0f;
        return;
    }

    self->accel_calibration_sum[0] += accel_x;
    self->accel_calibration_sum[1] += accel_y;
    self->accel_calibration_sum[2] += accel_z;
    self->accel_norm_calibration_sum += accel_norm;
    self->gyro_calibration_sum[0] += gyro_x;
    self->gyro_calibration_sum[1] += gyro_y;
    self->gyro_calibration_sum[2] += gyro_z;
    self->calibration_sample_count++;
    if (self->calibration_sample_count < CALIBRATION_SAMPLE_TARGET)
    {
        return;
    }

    {
        const float sample_count = (float)CALIBRATION_SAMPLE_TARGET;
        /* Uma unica posicao permite calibrar a escala, mas nao offsets por eixo
         * sem confundir a inclinacao real com erro do acelerometro. */
        const float mean_accel_norm = self->accel_norm_calibration_sum / sample_count;
        uint8_t axis;
        if (mean_accel_norm > 0.01f)
        {
            self->accel_scale = 1.0f / mean_accel_norm;
        }
        for (axis = 0U; axis < 3U; axis++)
        {
            self->gyro_offset[axis] = self->gyro_calibration_sum[axis] / sample_count;
        }
        self->calibration_complete = true;
        reset_filters(self);
    }
}

/**
 * @brief Atualiza a estimativa de roll/pitch/yaw (fusao complementar).
 *
 * Integra o giroscopio para o curto prazo e mistura o angulo do acelerometro
 * (quando a gravidade e confiavel) para anular a deriva no longo prazo. Aplica
 * filtro passa-baixa e rejeicao de picos no acelerometro antes de usa-lo. O peso
 * da mistura muda conforme ha movimento intencional, privilegiando o giroscopio
 * durante manobras e o acelerometro em repouso.
 *
 * @param self        Instancia do driver da IMU (estado do filtro).
 * @param data        Saida: atitude estimada (roll/pitch/yaw) e flags.
 * @param raw_accel_x Aceleracao bruta no eixo X (g), ja remapeada.
 * @param raw_accel_y Aceleracao bruta no eixo Y (g), ja remapeada.
 * @param raw_accel_z Aceleracao bruta no eixo Z (g), ja remapeada.
 * @param gyro_x      Velocidade angular no eixo X (graus/s).
 * @param gyro_y      Velocidade angular no eixo Y (graus/s).
 * @param gyro_z      Velocidade angular no eixo Z (graus/s).
 * @param now         Timestamp atual (millis) para calcular o dt do passo.
 */
static float update_attitude(mpu9259_t *self, mpu9259_data_t *data,
                             float raw_accel_x, float raw_accel_y, float raw_accel_z,
                             float gyro_x, float gyro_y, float gyro_z, uint32_t now)
{
    float dt_seconds = DEFAULT_DT_SECONDS;
    bool first_sample;
    bool spike_detected;
    float limited_accel_x;
    float limited_accel_y;
    float limited_accel_z;
    float alpha;
    float accel_norm_g;
    float filtered_gravity_magnitude;
    bool gravity_valid;
    float corrected_gyro_x;
    float corrected_gyro_y;
    float corrected_gyro_z;
    float gyro_roll_deg;
    float gyro_pitch_deg;
    bool intentional_motion;
    float gyro_weight;

    if (self->last_attitude_update_ms != 0U)
    {
        dt_seconds = (float)(now - self->last_attitude_update_ms) / 1000.0f;
        if ((dt_seconds <= 0.0f) || (dt_seconds > MAX_DT_SECONDS))
        {
            dt_seconds = DEFAULT_DT_SECONDS;
        }
    }
    self->last_attitude_update_ms = now;

    first_sample = !self->filter_initialized;
    if (first_sample)
    {
        self->filtered_accel[0] = raw_accel_x;
        self->filtered_accel[1] = raw_accel_y;
        self->filtered_accel[2] = raw_accel_z;
        self->filter_initialized = true;
    }

    spike_detected =
        (fabsf(raw_accel_x - self->filtered_accel[0]) > MAX_ACCEL_STEP_G) ||
        (fabsf(raw_accel_y - self->filtered_accel[1]) > MAX_ACCEL_STEP_G) ||
        (fabsf(raw_accel_z - self->filtered_accel[2]) > MAX_ACCEL_STEP_G);
    data->accel_sample_rejected = spike_detected;
    if (spike_detected && (self->rejected_sample_count < MAX_REJECTED_SAMPLE_COUNT))
    {
        self->rejected_sample_count++;
    }
    limited_accel_x = clamp_f(raw_accel_x, self->filtered_accel[0] - MAX_ACCEL_STEP_G,
                              self->filtered_accel[0] + MAX_ACCEL_STEP_G);
    limited_accel_y = clamp_f(raw_accel_y, self->filtered_accel[1] - MAX_ACCEL_STEP_G,
                              self->filtered_accel[1] + MAX_ACCEL_STEP_G);
    limited_accel_z = clamp_f(raw_accel_z, self->filtered_accel[2] - MAX_ACCEL_STEP_G,
                              self->filtered_accel[2] + MAX_ACCEL_STEP_G);
    alpha = clamp_f(low_pass_alpha(dt_seconds), 0.0f, 1.0f);
    self->filtered_accel[0] += alpha * (limited_accel_x - self->filtered_accel[0]);
    self->filtered_accel[1] += alpha * (limited_accel_y - self->filtered_accel[1]);
    self->filtered_accel[2] += alpha * (limited_accel_z - self->filtered_accel[2]);

    data->accel_x = self->filtered_accel[0];
    data->accel_y = self->filtered_accel[1];
    data->accel_z = self->filtered_accel[2];
    data->rejected_sample_count = self->rejected_sample_count;

    accel_norm_g = sqrtf((raw_accel_x * raw_accel_x) + (raw_accel_y * raw_accel_y) +
                         (raw_accel_z * raw_accel_z));
    filtered_gravity_magnitude = sqrtf((data->accel_x * data->accel_x) +
                                       (data->accel_y * data->accel_y) +
                                       (data->accel_z * data->accel_z));
    data->accel_norm_g = accel_norm_g;
    gravity_valid = (!spike_detected) && (accel_norm_g >= MIN_GRAVITY_G) &&
                    (accel_norm_g <= MAX_GRAVITY_G) && (filtered_gravity_magnitude > 0.01f);
    data->accel_correction_used = gravity_valid;

    corrected_gyro_x = remove_gyro_noise(gyro_x);
    corrected_gyro_y = remove_gyro_noise(gyro_y);
    corrected_gyro_z = remove_gyro_noise(gyro_z);
    gyro_roll_deg = self->roll_deg + (corrected_gyro_x * dt_seconds);
    gyro_pitch_deg = self->pitch_deg + (corrected_gyro_y * dt_seconds);
    self->yaw_deg = wrap_angle_180(self->yaw_deg + (corrected_gyro_z * dt_seconds));
    intentional_motion = (fabsf(corrected_gyro_x) >= MOTION_RATE_THRESHOLD_DPS) ||
                         (fabsf(corrected_gyro_y) >= MOTION_RATE_THRESHOLD_DPS);
    gyro_weight = intentional_motion ? MOVING_GYRO_WEIGHT : STABLE_GYRO_WEIGHT;
    if (gravity_valid)
    {
        const float normalized_x = data->accel_x / filtered_gravity_magnitude;
        const float normalized_y = data->accel_y / filtered_gravity_magnitude;
        const float normalized_z = data->accel_z / filtered_gravity_magnitude;
        const float accel_roll_deg = atan2f(normalized_y, normalized_z) * RADIANS_TO_DEGREES;
        const float accel_pitch_deg =
            atan2f(-normalized_x, sqrtf((normalized_y * normalized_y) +
                                        (normalized_z * normalized_z))) * RADIANS_TO_DEGREES;
        if (first_sample)
        {
            self->roll_deg = accel_roll_deg;
            self->pitch_deg = accel_pitch_deg;
        }
        else
        {
            self->roll_deg = (gyro_weight * gyro_roll_deg) + ((1.0f - gyro_weight) * accel_roll_deg);
            self->pitch_deg = (gyro_weight * gyro_pitch_deg) + ((1.0f - gyro_weight) * accel_pitch_deg);
        }
    }
    else
    {
        self->roll_deg = gyro_roll_deg;
        self->pitch_deg = gyro_pitch_deg;
    }

    data->roll_deg = self->roll_deg;
    data->pitch_deg = self->pitch_deg;
    data->yaw_deg = self->yaw_deg;
    data->dt_seconds = dt_seconds;
    return dt_seconds;
}

/**
 * @brief Corrige o yaw usando o magnetometro compensado por inclinacao.
 *
 * Projeta o campo magnetico no plano horizontal (usando roll/pitch atuais) para
 * obter o rumo e o aplica suavemente sobre o yaw integrado do giroscopio,
 * limitando a deriva. Na primeira leitura valida, adota o rumo medido direto.
 */
static void update_yaw_from_magnetometer(mpu9259_t *self, mpu9259_data_t *data)
{
    const float roll_rad = self->roll_deg * DEGREES_TO_RADIANS;
    const float pitch_rad = self->pitch_deg * DEGREES_TO_RADIANS;
    const float sin_roll = sinf(roll_rad);
    const float cos_roll = cosf(roll_rad);
    const float sin_pitch = sinf(pitch_rad);
    const float cos_pitch = cosf(pitch_rad);
    const float horizontal_x = (data->mag_x * cos_pitch) + (data->mag_z * sin_pitch);
    const float horizontal_y = (data->mag_x * sin_roll * sin_pitch) +
                               (data->mag_y * cos_roll) -
                               (data->mag_z * sin_roll * cos_pitch);
    const float magnetometer_yaw_deg = atan2f(horizontal_y, horizontal_x) * RADIANS_TO_DEGREES;
    if (!self->magnetometer_yaw_initialized)
    {
        self->yaw_deg = wrap_angle_180(magnetometer_yaw_deg);
        self->magnetometer_yaw_initialized = true;
    }
    else
    {
        const float yaw_error_deg = wrap_angle_180(magnetometer_yaw_deg - self->yaw_deg);
        self->yaw_deg = wrap_angle_180(self->yaw_deg + (MAGNETOMETER_CORRECTION_WEIGHT * yaw_error_deg));
    }
}

bool mpu9259_begin(mpu9259_t *self)
{
    uint8_t identity = 0U;
    size_t index;

    init_defaults(self);

    for (index = 0U; index < (sizeof(MPU_ADDRESSES) / sizeof(MPU_ADDRESSES[0])); index++)
    {
        self->address = MPU_ADDRESSES[index];
        if (read_registers(self->address, WHO_AM_I, &identity, 1U) &&
            ((identity == 0x71U) || (identity == 0x73U) || (identity == 0x68U)))
        {
            self->online = true;
            break;
        }
    }

    if (!self->online)
    {
        return false;
    }

    self->online = write_register(self->address, PWR_MGMT_1, 0x80U);
    vTaskDelay(pdMS_TO_TICKS(100));
    self->online = self->online && write_register(self->address, PWR_MGMT_1, 0x01U);
    self->online = self->online && write_register(self->address, REG_CONFIG, GYRO_DLPF_20_HZ);
    self->online = self->online && write_register(self->address, GYRO_CONFIG, 0x00U);
    self->online = self->online && write_register(self->address, ACCEL_CONFIG, 0x00U);
    self->online = self->online && write_register(self->address, ACCEL_CONFIG_2, ACCEL_DLPF_20_HZ);
    self->online = self->online && write_register(self->address, INT_PIN_CFG, 0x02U);
    self->magnetometer_available = self->online && initialize_magnetometer(self);
    reset_calibration(self);
    reset_filters(self);
    self->error_count = 0U;
    return self->online;
}

bool mpu9259_read(mpu9259_t *self, mpu9259_data_t *data)
{
    uint8_t raw[14];
    float raw_accel_x;
    float raw_accel_y;
    float raw_accel_z;
    float raw_gyro_x;
    float raw_gyro_y;
    float raw_gyro_z;
    float calibrated_accel_x;
    float calibrated_accel_y;
    float calibrated_accel_z;
    uint32_t now;

    data->valid = false;
    data->magnetometer_available = self->magnetometer_available;

    if (!self->online || !read_registers(self->address, ACCEL_XOUT_H, raw, sizeof(raw)))
    {
        self->error_count++;
        data->error_count = self->error_count;
        return false;
    }

    raw_accel_x = (float)decode_big_endian16(&raw[0]) / 16384.0f;
    raw_accel_y = (float)decode_big_endian16(&raw[2]) / 16384.0f;
    raw_accel_z = (float)decode_big_endian16(&raw[4]) / 16384.0f;
    data->temperature_c = ((float)decode_big_endian16(&raw[6]) / 333.87f) + 21.0f;
    raw_gyro_x = (float)decode_big_endian16(&raw[8]) / 131.0f;
    raw_gyro_y = (float)decode_big_endian16(&raw[10]) / 131.0f;
    raw_gyro_z = (float)decode_big_endian16(&raw[12]) / 131.0f;
#if (MPU_REMAP_ROTATE_Y_90 != 0)
    /* Compensa a orientacao fisica da IMU antes de qualquer uso (calibracao,
     * fusao de atitude). Accel e giroscopio sao rotacionados igualmente. */
    apply_axis_remap(&raw_accel_x, &raw_accel_y, &raw_accel_z);
    apply_axis_remap(&raw_gyro_x, &raw_gyro_y, &raw_gyro_z);
#endif
    update_calibration(self, raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, raw_gyro_y, raw_gyro_z);
    calibrated_accel_x = (raw_accel_x - self->accel_offset[0]) * self->accel_scale;
    calibrated_accel_y = (raw_accel_y - self->accel_offset[1]) * self->accel_scale;
    calibrated_accel_z = (raw_accel_z - self->accel_offset[2]) * self->accel_scale;
    data->gyro_x = raw_gyro_x - self->gyro_offset[0];
    data->gyro_y = raw_gyro_y - self->gyro_offset[1];
    data->gyro_z = raw_gyro_z - self->gyro_offset[2];
    data->calibration_complete = self->calibration_complete;
    data->calibration_progress =
        (uint8_t)((self->calibration_sample_count * 100U) / CALIBRATION_SAMPLE_TARGET);
    now = millis();
    (void)update_attitude(self, data, calibrated_accel_x, calibrated_accel_y, calibrated_accel_z,
                          data->gyro_x, data->gyro_y, data->gyro_z, now);

    if (self->magnetometer_available)
    {
        uint8_t status = 0U;
        if (read_registers(AK8963_ADDRESS, AK8963_ST1, &status, 1U) && ((status & 0x01U) != 0U))
        {
            uint8_t mag_raw[7];
            if (read_registers(AK8963_ADDRESS, AK8963_ST1 + 1U, mag_raw, sizeof(mag_raw)) &&
                ((mag_raw[6] & 0x08U) == 0U))
            {
                const float microtesla_per_lsb = 0.15f;
                data->mag_x = (float)decode_little_endian16(&mag_raw[0]) * microtesla_per_lsb * self->mag_adjustment[0];
                data->mag_y = (float)decode_little_endian16(&mag_raw[2]) * microtesla_per_lsb * self->mag_adjustment[1];
                data->mag_z = (float)decode_little_endian16(&mag_raw[4]) * microtesla_per_lsb * self->mag_adjustment[2];
#if (MPU_REMAP_ROTATE_Y_90 != 0)
                /* Leva o magnetometro ao mesmo frame corrigido de accel/giro.
                 * Combina o alinhamento AK8963->corpo (datasheet) com a rotacao
                 * fisica R_y(+90): mag_x=-ak_z; mag_y=ak_x; mag_z=-ak_y.
                 * Confirmado empiricamente: AK-Y e o eixo vertical desta montagem. */
                {
                    const float ak_x = data->mag_x;
                    const float ak_y = data->mag_y;
                    const float ak_z = data->mag_z;
                    data->mag_x = -ak_z;
                    data->mag_y = ak_x;
                    data->mag_z = -ak_y;
                }
#endif
                update_yaw_from_magnetometer(self, data);
            }
        }
    }
    data->yaw_deg = self->yaw_deg;

    self->error_count = 0U;
    data->error_count = 0U;
    data->updated_at_ms = now;
    self->sample_seq++;
    data->sample_seq = self->sample_seq;
    data->valid = true;
    return true;
}

bool mpu9259_is_online(const mpu9259_t *self)
{
    return self->online;
}
