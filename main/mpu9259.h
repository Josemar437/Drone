/**
 * @file mpu9259.h
 * @brief Driver da IMU MPU-9250/9255/9259 (acelerometro + giroscopio + AK8963).
 *
 * Le os sensores inerciais e entrega a atitude estimada (roll/pitch/yaw) ja
 * fundida, pronta para a malha de controle. Faz tres coisas que o controle
 * depende:
 *
 *  - Fusao complementar: combina o giroscopio (bom no curto prazo, mas com
 *    deriva) com o acelerometro (estavel no longo prazo, mas ruidoso) para um
 *    roll/pitch estavel; o yaw e integrado do giro e corrigido pelo magnetometro
 *    quando disponivel.
 *  - Autocalibracao em repouso: estima offsets do giroscopio e a escala do
 *    acelerometro enquanto a placa esta parada, eliminando vies de fabrica.
 *  - Rejeicao de picos: descarta amostras de acelerometro com saltos bruscos
 *    (vibracao/choque) para nao contaminar a atitude.
 *
 * Uso tipico: ::mpu9259_begin uma vez e ::mpu9259_read periodicamente (~50 Hz).
 */
#ifndef MPU9259_H
#define MPU9259_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Amostra de saida da IMU (atitude fundida + dados brutos + diagnostico).
 */
typedef struct
{
    float accel_x;                 /**< Aceleracao filtrada X (g). */
    float accel_y;                 /**< Aceleracao filtrada Y (g). */
    float accel_z;                 /**< Aceleracao filtrada Z (g). */
    float gyro_x;                  /**< Velocidade angular X, sem offset (graus/s). */
    float gyro_y;                  /**< Velocidade angular Y, sem offset (graus/s). */
    float gyro_z;                  /**< Velocidade angular Z, sem offset (graus/s). */
    float mag_x;                   /**< Campo magnetico X (uT), se disponivel. */
    float mag_y;                   /**< Campo magnetico Y (uT), se disponivel. */
    float mag_z;                   /**< Campo magnetico Z (uT), se disponivel. */
    float roll_deg;                /**< Rolagem estimada (fusao). */
    float pitch_deg;               /**< Arfagem estimada (fusao). */
    float yaw_deg;                 /**< Guinada estimada (giro + magnetometro). */
    float accel_norm_g;            /**< Modulo do vetor aceleracao bruto (g). */
    float temperature_c;           /**< Temperatura do chip (graus C). */
    bool valid;                    /**< true se esta amostra e confiavel. */
    bool magnetometer_available;   /**< true se o AK8963 foi detectado. */
    bool accel_correction_used;    /**< true se o acelerometro corrigiu a atitude. */
    bool accel_sample_rejected;    /**< true se esta amostra teve pico descartado. */
    bool calibration_complete;     /**< true apos a autocalibracao terminar. */
    uint8_t calibration_progress;  /**< Progresso da calibracao (0..100 %). */
    uint32_t updated_at_ms;        /**< Timestamp da amostra (millis). */
    uint16_t error_count;          /**< Falhas de leitura consecutivas. */
    uint16_t rejected_sample_count;/**< Total de amostras rejeitadas. */
} mpu9259_data_t;

/**
 * @brief Estado interno do driver (filtros, calibracao, enderecos).
 *
 * Mantido entre chamadas de ::mpu9259_read. Inicialize com ::mpu9259_begin
 * antes de ler. Os campos sao detalhados na implementacao.
 */
typedef struct
{
    uint8_t address;
    bool online;
    bool magnetometer_available;
    float mag_adjustment[3];
    float filtered_accel[3];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float accel_offset[3];
    float accel_scale;
    float gyro_offset[3];
    float accel_calibration_sum[3];
    float accel_norm_calibration_sum;
    float gyro_calibration_sum[3];
    uint16_t calibration_sample_count;
    bool calibration_complete;
    bool magnetometer_yaw_initialized;
    uint32_t last_attitude_update_ms;
    uint16_t rejected_sample_count;
    bool filter_initialized;
    uint16_t error_count;
} mpu9259_t;

/**
 * @brief Detecta e configura a IMU; zera filtros e calibracao.
 *
 * Requer o barramento I2C ja inicializado. Varre os enderecos 0x68/0x69,
 * verifica o WHO_AM_I, configura faixas/filtros e tenta iniciar o magnetometro.
 *
 * @param self Instancia a inicializar.
 * @return true se a IMU foi encontrada e configurada; false caso contrario.
 */
bool mpu9259_begin(mpu9259_t *self);

/**
 * @brief Le uma amostra, atualiza a fusao de atitude e a calibracao.
 *
 * Deve ser chamada periodicamente (o dt e medido internamente via millis).
 *
 * @param self Instancia previamente inicializada.
 * @param data Saida preenchida com a amostra (e diagnostico).
 * @return true se a leitura foi valida; false em falha de I2C (data.valid=false).
 *
 * @code
 * mpu9259_data_t s;
 * if (mpu9259_read(&imu, &s) && s.calibration_complete) {
 *     // usar s.roll_deg, s.pitch_deg, s.gyro_x ...
 * }
 * @endcode
 */
bool mpu9259_read(mpu9259_t *self, mpu9259_data_t *data);

/**
 * @brief Indica se a IMU esta online (detectada e respondendo).
 * @return true se online.
 */
bool mpu9259_is_online(const mpu9259_t *self);

#endif /* MPU9259_H */
