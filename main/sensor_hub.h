/**
 * @file sensor_hub.h
 * @brief Agregador dos sensores (IMU, barometro, GPS) com acesso seguro entre tarefas.
 *
 * Centraliza a leitura periodica dos tres sensores e publica "snapshots"
 * consistentes para o resto do sistema. O problema que resolve: os sensores sao
 * lidos por uma tarefa (telemetria) e consumidos por outra (controle de voo) e
 * pelo servidor web; sem coordenacao, um leitor poderia pegar uma estrutura
 * sendo escrita pela metade. Por isso as copias entre produtor e consumidores
 * acontecem dentro de uma secao critica (portMUX).
 *
 * Tambem cuida das cadencias por sensor (IMU rapida, barometro lento, GPS
 * continuo) e tenta reconectar sensores que sairam do ar.
 *
 * Uso: ::sensor_hub_init, ::sensor_hub_begin (uma vez), ::sensor_hub_update em
 * laco; consumidores usam os *_snapshot e ::sensor_hub_to_json.
 */
#ifndef SENSOR_HUB_H
#define SENSOR_HUB_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "mpu9259.h"
#include "bmp280.h"
#include "neo6m_gps.h"

/** @brief Capacidade recomendada do buffer para ::sensor_hub_to_json. */
#define SENSOR_HUB_JSON_CAPACITY (1300)

/**
 * @brief Estado do agregador: sensores, ultimos snapshots, mutex e cadencias.
 */
typedef struct
{
    mpu9259_t mpu;             /**< Driver da IMU. */
    bmp280_t bmp;              /**< Driver do barometro. */
    neo6m_gps_t gps;           /**< Driver do GPS. */
    mpu9259_data_t mpu_data;   /**< Ultimo snapshot da IMU. */
    bmp280_data_t bmp_data;    /**< Ultimo snapshot do barometro. */
    neo6m_data_t gps_data;     /**< Ultimo snapshot do GPS. */
    portMUX_TYPE data_mutex;   /**< Protege as copias entre tarefas. */
    uint32_t last_mpu_read_ms; /**< Controle de cadencia da IMU. */
    uint32_t last_bmp_read_ms; /**< Controle de cadencia do barometro. */
    uint32_t last_retry_ms;    /**< Controle da tentativa de reconexao. */
} sensor_hub_t;

/**
 * @brief Zera o estado e inicializa o mutex. Chamar antes de ::sensor_hub_begin.
 */
void sensor_hub_init(sensor_hub_t *self);

/**
 * @brief Inicializa o barramento I2C, o GPS e detecta os sensores.
 *
 * @param self          Instancia (ja passada por ::sensor_hub_init).
 * @param sda_gpio      GPIO SDA do I2C.
 * @param scl_gpio      GPIO SCL do I2C.
 * @param gps_uart_port Porta UART do GPS (ex.: UART_NUM_2).
 * @param gps_rx_gpio   GPIO RX do GPS.
 * @param gps_tx_gpio   GPIO TX do GPS.
 */
void sensor_hub_begin(sensor_hub_t *self, int sda_gpio, int scl_gpio,
                      int gps_uart_port, int gps_rx_gpio, int gps_tx_gpio);

/**
 * @brief Le os sensores nas suas cadencias e publica os snapshots.
 *
 * Chamar com frequencia alta (ex.: a cada 2 ms): a funcao decide internamente
 * quando cada sensor deve ser lido. Tambem tenta reconectar sensores offline.
 */
void sensor_hub_update(sensor_hub_t *self);

/**
 * @brief Monta a telemetria completa (IMU+BMP+GPS) em JSON.
 *
 * @param self Instancia.
 * @param out  Buffer de saida.
 * @param cap  Capacidade de 'out' (use ::SENSOR_HUB_JSON_CAPACITY).
 * @return Numero de bytes escritos (sem o terminador).
 */
size_t sensor_hub_to_json(sensor_hub_t *self, char *out, size_t cap);

/**
 * @brief Copia o ultimo snapshot da IMU de forma atomica.
 * @param self Instancia.
 * @param data Saida.
 * @return data->valid (true se o snapshot e confiavel).
 *
 * @code
 * mpu9259_data_t imu;
 * if (sensor_hub_get_mpu_snapshot(hub, &imu) && imu.calibration_complete) { ... }
 * @endcode
 */
bool sensor_hub_get_mpu_snapshot(sensor_hub_t *self, mpu9259_data_t *data);

/**
 * @brief Copia o ultimo snapshot do barometro de forma atomica.
 * @return data->valid.
 */
bool sensor_hub_get_bmp_snapshot(sensor_hub_t *self, bmp280_data_t *data);

/**
 * @brief Copia o ultimo snapshot do GPS de forma atomica.
 * @return data->valid.
 */
bool sensor_hub_get_gps_snapshot(sensor_hub_t *self, neo6m_data_t *data);

#endif /* SENSOR_HUB_H */
