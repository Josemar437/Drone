/**
 * @file neo6m_gps.h
 * @brief Driver do GPS NEO-6M: le e interpreta sentencas NMEA pela UART.
 *
 * O modulo GPS envia texto NMEA continuamente. Este driver acumula os bytes da
 * UART, valida o checksum de cada sentenca e extrai posicao/velocidade das
 * sentencas GGA (fix + altitude + satelites) e RMC (posicao + velocidade).
 *
 * Tambem detecta perda de sinal por tempo: se nao chega fix valido dentro de um
 * limite, marca os dados como invalidos (signal_lost), evitando que a aplicacao
 * use uma posicao velha como se fosse atual.
 *
 * Uso tipico: ::neo6m_begin uma vez e ::neo6m_update com frequencia (drena a
 * UART); leia o resultado com ::neo6m_data.
 */
#ifndef NEO6M_GPS_H
#define NEO6M_GPS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Dados de navegacao extraidos do GPS.
 */
typedef struct
{
    double latitude;          /**< Latitude em graus decimais (negativo = Sul). */
    double longitude;         /**< Longitude em graus decimais (negativo = Oeste). */
    float altitude_m;         /**< Altitude (m) reportada pela GGA. */
    float speed_kmh;          /**< Velocidade sobre o solo (km/h). */
    float hdop;               /**< Diluicao horizontal de precisao (menor = melhor). */
    uint8_t satellites;       /**< Numero de satelites em uso. */
    uint8_t fix_quality;      /**< Qualidade do fix (0 = sem fix). */
    bool valid;               /**< true se ha posicao valida e recente. */
    bool signal_lost;         /**< true se o sinal expirou por tempo. */
    uint32_t updated_at_ms;   /**< Timestamp do ultimo fix valido (millis). */
    uint16_t checksum_errors; /**< Sentencas descartadas por checksum invalido. */
} neo6m_data_t;

/**
 * @brief Estado interno do driver (porta UART, ultimo dado e buffer de linha).
 *
 * Inicialize com ::neo6m_begin antes de usar.
 */
typedef struct
{
    int uart_port;       /**< Porta UART (ex.: UART_NUM_2). */
    neo6m_data_t data;   /**< Ultimo dado consolidado. */
    char line[100];      /**< Buffer de montagem da sentenca atual. */
    size_t line_length;  /**< Bytes acumulados em 'line'. */
} neo6m_gps_t;

/**
 * @brief Configura a UART do GPS e zera o estado.
 *
 * @param self      Instancia a inicializar.
 * @param uart_port Porta UART (0/1/2).
 * @param baud      Taxa (NEO-6M de fabrica: 9600).
 * @param rx_gpio   GPIO de recepcao (RX do ESP32, ligado ao TX do GPS).
 * @param tx_gpio   GPIO de transmissao (normalmente nao usado pelo GPS).
 */
void neo6m_begin(neo6m_gps_t *self, int uart_port, uint32_t baud, int rx_gpio, int tx_gpio);

/**
 * @brief Drena os bytes disponiveis da UART, processa sentencas e atualiza o timeout.
 *
 * Nao bloqueia: processa apenas o que ja chegou. Chame com frequencia suficiente
 * para nao deixar o buffer da UART encher.
 *
 * @param self Instancia.
 */
void neo6m_update(neo6m_gps_t *self);

/**
 * @brief Retorna uma copia do ultimo dado consolidado.
 * @return Copia de ::neo6m_data_t (verifique .valid / .signal_lost antes de usar).
 */
neo6m_data_t neo6m_data(const neo6m_gps_t *self);

#endif /* NEO6M_GPS_H */
