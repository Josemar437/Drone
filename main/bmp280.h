/**
 * @file bmp280.h
 * @brief Driver do barometro BMP280 (pressao, temperatura e altitude relativa).
 *
 * Mede pressao/temperatura e deriva a altitude. Como a pressao absoluta varia
 * com o clima, o que interessa ao voo e a altitude *relativa* ao ponto de
 * partida: o driver aprende uma pressao de referencia logo apos ligar (placa
 * parada) e reporta a altura em relacao a ela, com filtragem e rejeicao de
 * picos para uma leitura estavel.
 *
 * A compensacao de pressao/temperatura usa o algoritmo oficial da Bosch
 * (aritmetica inteira de 64 bits) com os coeficientes de calibracao do chip.
 *
 * Uso tipico: ::bmp280_begin uma vez e ::bmp280_read periodicamente (~5 Hz).
 */
#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Amostra do barometro (pressao, temperatura, altitudes e diagnostico).
 */
typedef struct
{
    float temperature_c;            /**< Temperatura compensada (graus C). */
    float pressure_hpa;             /**< Pressao compensada (hPa). */
    float pressure_reference_hpa;   /**< Pressao de referencia aprendida (hPa). */
    float altitude_absolute_m;      /**< Altitude vs nivel do mar padrao (m). */
    float altitude_m;               /**< Altitude relativa filtrada (m). */
    float altitude_reference_m;     /**< Altitude da referencia (m). */
    bool valid;                     /**< true se a amostra e confiavel. */
    bool reference_ready;           /**< true apos aprender a referencia. */
    bool altitude_sample_rejected;  /**< true se esta amostra teve pico descartado. */
    uint8_t reference_progress;     /**< Progresso do aprendizado (0..100 %). */
    uint32_t updated_at_ms;         /**< Timestamp da amostra (millis). */
    uint16_t error_count;           /**< Falhas de leitura consecutivas. */
    uint16_t rejected_sample_count; /**< Total de amostras rejeitadas. */
} bmp280_data_t;

/**
 * @brief Estado interno do driver (coeficientes de calibracao, filtro, referencia).
 *
 * Mantido entre chamadas de ::bmp280_read. Inicialize com ::bmp280_begin.
 */
typedef struct
{
    uint8_t address;
    bool online;
    uint16_t error_count;
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    int32_t fine_temperature;
    float pressure_reference_sum_hpa;
    float pressure_reference_hpa;
    float altitude_reference_m;
    float filtered_relative_altitude_m;
    float previous_raw_relative_altitude_m;
    uint16_t stabilization_sample_count;
    uint16_t reference_sample_count;
    uint16_t rejected_sample_count;
    bool altitude_filter_initialized;
    bool reference_ready;
} bmp280_t;

/**
 * @brief Detecta o BMP280, le os coeficientes de calibracao e configura o modo.
 *
 * Requer o barramento I2C ja inicializado. Varre os enderecos 0x76/0x77 e
 * verifica o chip ID.
 *
 * @param self Instancia a inicializar.
 * @return true se o sensor foi encontrado e configurado; false caso contrario.
 */
bool bmp280_begin(bmp280_t *self);

/**
 * @brief Le pressao/temperatura, compensa e atualiza a altitude relativa.
 *
 * @param self          Instancia previamente inicializada.
 * @param data          Saida preenchida com a amostra.
 * @param sea_level_hpa Pressao de referencia ao nivel do mar (ex.: 1013.25),
 *                      usada apenas para a altitude absoluta.
 * @return true se a leitura foi valida; false em falha/leitura fora de faixa.
 *
 * @code
 * bmp280_data_t s;
 * if (bmp280_read(&baro, &s, 1013.25f) && s.reference_ready) {
 *     // s.altitude_m = altura relativa ao ponto de partida
 * }
 * @endcode
 */
bool bmp280_read(bmp280_t *self, bmp280_data_t *data, float sea_level_hpa);

/**
 * @brief Indica se o barometro esta online (detectado e respondendo).
 * @return true se online.
 */
bool bmp280_is_online(const bmp280_t *self);

#endif /* BMP280_H */
