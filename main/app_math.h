/**
 * @file app_math.h
 * @brief Utilitarios aritmeticos de saturacao e mapeamento linear.
 *
 * Implementam a saturacao por limites e o mapeamento linear em aritmetica
 * inteira (com truncamento na divisao) usados em varios pontos da cadeia de PWM.
 *
 * Sao funcoes inline em vez de macros porque macros do tipo funcao avaliam os
 * argumentos mais de uma vez (efeitos colaterais duplicados) e nao tem
 * verificacao de tipo; funcoes inline evitam ambos sem custo de chamada.
 */
#ifndef APP_MATH_H
#define APP_MATH_H

#include <stdint.h>

/**
 * @brief Satura um inteiro de 32 bits ao intervalo [low, high].
 *
 * Usado em toda saida de PWM para garantir que nenhum pulso saia da faixa
 * valida do ESC.
 *
 * @param value Valor a saturar.
 * @param low   Limite inferior (inclusive).
 * @param high  Limite superior (inclusive).
 * @return value limitado a [low, high]. O chamador garante low <= high.
 *
 * @code
 * int32_t pulso = clamp_i32(comando_us, 1000, 2000); // nunca fora de 1000..2000
 * @endcode
 */
static inline int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
    int32_t result = value;
    if (result < low)
    {
        result = low;
    }
    else if (result > high)
    {
        result = high;
    }
    else
    {
        result = value;
    }
    return result;
}

/**
 * @brief Satura um float ao intervalo [low, high].
 *
 * Versao em ponto flutuante de clamp_i32, usada nos controladores PID para
 * limitar integral e saida, e na fusao de sensores para limitar passos.
 *
 * @param value Valor a saturar.
 * @param low   Limite inferior (inclusive).
 * @param high  Limite superior (inclusive).
 * @return value limitado a [low, high]. O chamador garante low <= high.
 *
 * @code
 * integral = clamp_f(integral, -integral_limit, integral_limit); // anti-windup
 * @endcode
 */
static inline float clamp_f(float value, float low, float high)
{
    float result = value;
    if (result < low)
    {
        result = low;
    }
    else if (result > high)
    {
        result = high;
    }
    else
    {
        result = value;
    }
    return result;
}

/**
 * @brief Mapeia linearmente x de [in_low, in_high] para [out_low, out_high].
 *
 * Mapeamento em aritmetica inteira: a divisao trunca em direcao a zero (nao
 * arredonda). Esse truncamento faz parte da definicao da saida do motor e por
 * isso deve ser preservado exatamente.
 *
 * @param x        Valor de entrada.
 * @param in_low   Inicio da faixa de entrada.
 * @param in_high  Fim da faixa de entrada (deve ser diferente de in_low).
 * @param out_low  Inicio da faixa de saida.
 * @param out_high Fim da faixa de saida.
 * @return x reescalado para a faixa de saida.
 * @warning Divisao por zero se in_high == in_low; o chamador deve garantir
 *          faixas validas (na cadeia de PWM, in_high - in_low = 999).
 *
 * @code
 * // Mapeia o pulso logico (1001..2000) para a faixa fisica do motor:
 * int32_t saida_us = map_i32(logico_us, 1001, 2000, start_us, max_us);
 * @endcode
 */
static inline int32_t map_i32(int32_t x, int32_t in_low, int32_t in_high,
                              int32_t out_low, int32_t out_high)
{
    return (((x - in_low) * (out_high - out_low)) / (in_high - in_low)) + out_low;
}

#endif /* APP_MATH_H */
