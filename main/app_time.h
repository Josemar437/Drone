/**
 * @file app_time.h
 * @brief Relogio em milissegundos desde o boot (funcao millis()).
 *
 * Diversos modulos (sensores, fusao, watchdog de comando) medem intervalos com
 * millis(). Em vez de espalhar conversoes de esp_timer pelo codigo, centraliza-se
 * aqui uma unica funcao: contagem crescente desde o boot que envolve em
 * ~49,7 dias (uint32_t).
 *
 * O envolvimento (wrap) e seguro para os usos do firmware porque todos calculam
 * diferencas (now - antes), e a subtracao em uint32_t produz o intervalo correto
 * mesmo quando o contador envolve.
 */
#ifndef APP_TIME_H
#define APP_TIME_H

#include <stdint.h>
#include "esp_timer.h"

/**
 * @brief Retorna milissegundos decorridos desde o boot.
 *
 * @return Tempo em ms (uint32_t).
 *
 * @code
 * const uint32_t inicio = millis();
 * // ... trabalho ...
 * if ((millis() - inicio) > 600U) { // intervalo correto mesmo no wrap
 *     // tempo esgotado
 * }
 * @endcode
 */
static inline uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / (int64_t)1000);
}

#endif /* APP_TIME_H */
