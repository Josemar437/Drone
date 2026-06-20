/**
 * @file esc_pwm.h
 * @brief Geracao de PWM para os 4 ESCs (via LEDC) e logica de calibracao.
 *
 * Encapsula tudo que escreve pulso nos ESCs, gerando via LEDC pulsos de
 * 1000..2000 us a 250 Hz: para um mesmo valor logico, a largura de pulso em
 * microssegundos enviada ao ESC e consistente.
 *
 * Conceitos:
 *  - "valor logico" (us): 1000..2000, onde 1000 = parado. E o que o resto do
 *    sistema manipula (mixagem, sliders da interface).
 *  - "saida calibrada" (us): o valor logico mapeado para a faixa util de cada
 *    motor (start..max + trim), compensando diferencas entre motores/ESCs.
 *
 * O modulo guarda internamente a calibracao e a ultima velocidade/saida de cada
 * motor (singleton): nao ha instancia a passar, apenas chame as funcoes.
 */
#ifndef ESC_PWM_H
#define ESC_PWM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Configura o timer e os 4 canais LEDC e poe todos em ESC_MIN.
 *
 * Deve ser chamada uma vez no boot, antes de qualquer escrita. Apos ela, e
 * necessario manter o sinal minimo pelo tempo de arming (ver ESC_ARM_HOLD_MS)
 * para os ESCs habilitarem.
 *
 * @return ESP_OK se timer e canais foram configurados; codigo de erro do LEDC
 *         caso contrario.
 */
esp_err_t esc_pwm_init(void);

/**
 * @brief Forca todos os motores ao sinal minimo (ESC_MIN).
 *
 * Usada no arming e como rede de seguranca em paradas.
 */
void esc_pwm_set_all_min(void);

/**
 * @brief Converte um valor logico (us) na saida calibrada do motor (us).
 *
 * Funcao pura (nao escreve no hardware). Aplica start/max/trim e o piso de
 * operacao. Valores <= ESC_MIN retornam ESC_MIN (motor parado).
 *
 * @param motor      Indice 0..NUM_MOTORS-1.
 * @param command_us Valor logico (sera saturado a 1000..2000).
 * @return Pulso calibrado a enviar ao ESC; ESC_MIN se motor invalido/parado.
 */
int32_t esc_calibrate_motor_output(uint8_t motor, int32_t command_us);

/**
 * @brief Define a velocidade logica de um motor, calibra e escreve no ESC.
 *
 * Variante com log no console (uso interativo: testes, calibracao, interface).
 *
 * @param motor Indice 0..NUM_MOTORS-1 (fora da faixa: no-op).
 * @param speed Valor logico em us (sera saturado a 1000..2000).
 */
void esc_set_motor_speed(uint8_t motor, int32_t speed);

/**
 * @brief Igual a ::esc_set_motor_speed, porem sem log.
 *
 * Usada pelo laco de controle de 20 ms, onde imprimir no console a cada motor e
 * a cada ciclo prejudicaria o tempo real.
 *
 * @param motor Indice 0..NUM_MOTORS-1 (fora da faixa: no-op).
 * @param speed Valor logico em us (sera saturado a 1000..2000).
 */
void esc_set_motor_speed_quiet(uint8_t motor, int32_t speed);

/**
 * @brief Para todos os motores (ESC_MIN) e emite log.
 */
void esc_stop_all_motors(void);

/**
 * @brief Recalcula e reaplica a saida de todos os motores.
 *
 * Chamar apos alterar a calibracao para que a mudanca tenha efeito imediato sem
 * mudar a velocidade logica corrente.
 */
void esc_reapply_current_outputs(void);

/**
 * @brief Le a velocidade logica corrente de um motor.
 * @return Velocidade em us; ESC_MIN se motor invalido.
 */
int32_t esc_get_motor_speed(uint8_t motor);

/**
 * @brief Le a ultima saida calibrada (us) escrita no ESC.
 * @return Pulso em us; ESC_MIN se motor invalido.
 */
int32_t esc_get_motor_output_us(uint8_t motor);

/**
 * @brief Sanitiza a calibracao de um motor para limites coerentes.
 *
 * Garante trim em [-100, 100], start em [1000, 1999] e max em [start+1, 2000].
 * Evita configuracoes invalidas vindas da interface ou da NVS.
 */
void esc_sanitize_calibration(uint8_t motor);

/**
 * @brief Restaura a calibracao de todos os motores aos defaults de fabrica.
 * @note Apenas em memoria; persista com calibration_store_save() se desejar.
 */
void esc_reset_calibration_to_defaults(void);

/**
 * @brief Le a calibracao (start/max/trim) de um motor.
 * @param motor    Indice 0..NUM_MOTORS-1.
 * @param start_us Saida: pulso de partida (us). Ignorado se NULL.
 * @param max_us   Saida: topo da faixa (us). Ignorado se NULL.
 * @param trim_us  Saida: ajuste fino (us). Ignorado se NULL.
 */
void esc_get_calibration(uint8_t motor, int32_t *start_us, int32_t *max_us, int32_t *trim_us);

/**
 * @brief Define a calibracao de um motor (com sanitizacao automatica).
 */
void esc_set_calibration(uint8_t motor, int32_t start_us, int32_t max_us, int32_t trim_us);

/**
 * @brief Define o piso de operacao global (us). 0 desativa.
 *
 * Quando > 0, eleva o start efetivo de todos os motores, mantendo-os acima da
 * regiao de baixa rotacao (nao-linear). Util em operacao continua.
 */
void esc_set_operating_idle_us(int32_t idle_us);

/** @brief Le o piso de operacao global (us). */
int32_t esc_get_operating_idle_us(void);

/**
 * @brief Indica se uma varredura de deadband esta em andamento.
 * @return true durante ::esc_run_deadband_sweep (rejeitar novas chamadas).
 */
bool esc_deadband_sweep_active(void);

/**
 * @brief Sobe o pulso de UM motor em degraus para caracterizar o deadband.
 *
 * Procedimento de bancada: mantem os demais motores em ESC_MIN e eleva o pulso
 * bruto (sem calibracao) do motor escolhido, registrando cada degrau no log.
 * O operador observa em que pulso o motor comeca a girar. E BLOQUEANTE.
 *
 * @param motor    Indice 0..NUM_MOTORS-1.
 * @param from_us  Pulso inicial (saturado a 1000..2000).
 * @param to_us    Pulso final (>= from_us).
 * @param step_us  Incremento por degrau (1..50).
 * @param dwell_ms Tempo em cada degrau (50..2000).
 * @warning Execute SEMPRE SEM HELICES.
 */
void esc_run_deadband_sweep(uint8_t motor, int32_t from_us, int32_t to_us,
                            int32_t step_us, int32_t dwell_ms);

#endif /* ESC_PWM_H */
