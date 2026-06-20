/**
 * @file neo6m_gps.c
 * @brief Implementacao do driver GPS NEO-6M (ver neo6m_gps.h).
 *
 * Desvio MISRA (Rule 21.7): em vez de atoi/atof usa-se strtol/strtof, que sao
 * conformes e mais robustos a entradas malformadas. Para
 * campos NMEA bem formados (garantidos pela validacao de checksum) o resultado
 * numerico e equivalente.
 */
#include "neo6m_gps.h"

#include <stdlib.h>
#include <string.h>
#include "app_time.h"
#include "driver/uart.h"

#define GPS_SIGNAL_TIMEOUT_MS (3000U)
#define MAX_BYTES_PER_UPDATE  (256)
#define GPS_UART_RX_BUFFER     (1024)
#define GPS_MAX_FIELDS         (20)

/**
 * @brief Valida o checksum NMEA de uma sentenca ($....*HH).
 *
 * Faz o XOR de todos os bytes entre '$' e '*' e compara com o valor hexadecimal
 * apos o '*'. Descarta sentencas corrompidas na transmissao serial.
 *
 * @return true se o checksum confere.
 */
static bool validate_checksum(const char *sentence)
{
    const char *star;
    const char *p;
    uint8_t checksum = 0U;

    if ((sentence == NULL) || (sentence[0] != '$'))
    {
        return false;
    }
    star = strchr(sentence, '*');
    if ((star == NULL) || (strlen(star) < 3U))
    {
        return false;
    }
    for (p = sentence + 1; p < star; p++)
    {
        checksum ^= (uint8_t)(*p);
    }
    return (uint32_t)checksum == strtoul(star + 1, NULL, 16);
}

/**
 * @brief Converte coordenada NMEA (ddmm.mmmm) em graus decimais.
 *
 * O NMEA mistura graus e minutos no mesmo campo (graus*100 + minutos). Esta
 * funcao separa as partes e aplica o sinal do hemisferio (S/W ficam negativos).
 *
 * @param value      Campo numerico da sentenca.
 * @param hemisphere "N"/"S"/"E"/"W".
 * @return Coordenada em graus decimais (0 se o campo estiver vazio).
 */
static double parse_coordinate(const char *value, const char *hemisphere)
{
    double raw;
    int32_t degrees;
    double coordinate;

    if ((value == NULL) || (value[0] == '\0'))
    {
        return 0.0;
    }
    raw = strtod(value, NULL);
    degrees = (int32_t)(raw / 100.0);
    coordinate = (double)degrees + ((raw - ((double)degrees * 100.0)) / 60.0);
    if ((hemisphere != NULL) && ((hemisphere[0] == 'S') || (hemisphere[0] == 'W')))
    {
        coordinate = -coordinate;
    }
    return coordinate;
}

/**
 * @brief Interpreta a sentenca GGA (fix, satelites, HDOP, posicao, altitude).
 *
 * E a fonte de altitude e qualidade do fix. Se fix_quality == 0, marca os dados
 * como invalidos (sem posicao confiavel).
 */
static void parse_gga(neo6m_gps_t *self, char **fields, int count)
{
    if (count < 10)
    {
        return;
    }
    self->data.fix_quality = (uint8_t)strtol(fields[6], NULL, 10);
    self->data.satellites = (uint8_t)strtol(fields[7], NULL, 10);
    self->data.hdop = (fields[8][0] != '\0') ? strtof(fields[8], NULL) : 99.9f;
    if (self->data.fix_quality == 0U)
    {
        self->data.valid = false;
        return;
    }
    self->data.latitude = parse_coordinate(fields[2], fields[3]);
    self->data.longitude = parse_coordinate(fields[4], fields[5]);
    self->data.altitude_m = strtof(fields[9], NULL);
    self->data.updated_at_ms = millis();
    self->data.valid = true;
    self->data.signal_lost = false;
}

/**
 * @brief Interpreta a sentenca RMC (status, posicao, velocidade sobre o solo).
 *
 * Complementa a GGA com a velocidade. So usa os dados se o status for 'A'
 * (ativo/valido); 'V' indica dados invalidos.
 */
static void parse_rmc(neo6m_gps_t *self, char **fields, int count)
{
    if ((count < 8) || (fields[2][0] != 'A'))
    {
        return;
    }
    self->data.latitude = parse_coordinate(fields[3], fields[4]);
    self->data.longitude = parse_coordinate(fields[5], fields[6]);
    self->data.speed_kmh = strtof(fields[7], NULL) * 1.852f;
    self->data.updated_at_ms = millis();
    self->data.valid = true;
    self->data.signal_lost = false;
}

/**
 * @brief Valida e despacha uma sentenca NMEA completa.
 *
 * Confere o checksum, divide a sentenca em campos (substituindo as virgulas por
 * terminadores) e chama o interpretador conforme o tipo (GGA/RMC). Conta as
 * sentencas com checksum invalido para diagnostico.
 *
 * @note Modifica 'sentence' in-place (insere '\0' nos separadores).
 */
static void parse_sentence(neo6m_gps_t *self, char *sentence)
{
    char *checksum_marker;
    char *fields[GPS_MAX_FIELDS];
    int count = 0;
    char *current = sentence;

    if (!validate_checksum(sentence))
    {
        self->data.checksum_errors++;
        return;
    }

    checksum_marker = strchr(sentence, '*');
    if (checksum_marker != NULL)
    {
        *checksum_marker = '\0';
    }

    while ((current != NULL) && (count < GPS_MAX_FIELDS))
    {
        char *comma;
        fields[count] = current;
        count++;
        comma = strchr(current, ',');
        if (comma == NULL)
        {
            break;
        }
        *comma = '\0';
        current = comma + 1;
    }

    if ((count > 0) && (strstr(fields[0], "GGA") != NULL))
    {
        parse_gga(self, fields, count);
    }
    if ((count > 0) && (strstr(fields[0], "RMC") != NULL))
    {
        parse_rmc(self, fields, count);
    }
}

void neo6m_begin(neo6m_gps_t *self, int uart_port, uint32_t baud, int rx_gpio, int tx_gpio)
{
    const uart_config_t config = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    (void)memset(&self->data, 0, sizeof(self->data));
    self->data.signal_lost = true;
    self->data.hdop = 99.9f;
    self->line_length = 0U;
    self->uart_port = uart_port;

    (void)uart_driver_install(uart_port, GPS_UART_RX_BUFFER, 0, 0, NULL, 0);
    (void)uart_param_config(uart_port, &config);
    (void)uart_set_pin(uart_port, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void neo6m_update(neo6m_gps_t *self)
{
    uint8_t buffer[MAX_BYTES_PER_UPDATE];
    int read_count;
    int index;

    read_count = uart_read_bytes(self->uart_port, buffer, (uint32_t)MAX_BYTES_PER_UPDATE, 0);
    for (index = 0; index < read_count; index++)
    {
        const char c = (char)buffer[index];
        if (c == '$')
        {
            self->line_length = 0U;
            self->line[self->line_length] = c;
            self->line_length++;
        }
        else if (c == '\n')
        {
            if (self->line_length > 6U)
            {
                self->line[self->line_length] = '\0';
                parse_sentence(self, self->line);
            }
            self->line_length = 0U;
        }
        else if ((c != '\r') && (self->line_length > 0U) &&
                 (self->line_length < (sizeof(self->line) - 1U)))
        {
            self->line[self->line_length] = c;
            self->line_length++;
        }
        else
        {
            /* byte ignorado fora de sentenca */
        }
    }

    self->data.signal_lost = (self->data.updated_at_ms == 0U) ||
                             ((millis() - self->data.updated_at_ms) > GPS_SIGNAL_TIMEOUT_MS);
    if (self->data.signal_lost)
    {
        self->data.valid = false;
    }
}

neo6m_data_t neo6m_data(const neo6m_gps_t *self)
{
    return self->data;
}
