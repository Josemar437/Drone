/**
 * @file calibration_store.h
 * @brief Persistencia da calibracao dos motores na NVS (memoria nao-volatil).
 *
 * A calibracao (start/max/trim por motor) e medida na bancada e precisa
 * sobreviver a desligamentos. Este modulo grava e le esses valores da NVS,
 * usando o namespace "motor-cal" e as chaves start%d/max%d/trim%d.
 *
 * O modulo nao guarda os valores: ele apenas faz a ponte entre a NVS e o
 * esc_pwm (fonte de verdade da calibracao em memoria).
 */
#ifndef CALIBRATION_STORE_H
#define CALIBRATION_STORE_H

#include "esp_err.h"

/**
 * @brief Inicializa o subsistema NVS. Chamar uma vez no boot, antes de load/save.
 *
 * Se a particao NVS estiver corrompida ou de versao incompativel, ela e apagada
 * e reinicializada (a calibracao volta ao default, melhor que falhar o boot).
 *
 * @return ESP_OK se a NVS esta utilizavel; codigo de erro caso contrario.
 */
esp_err_t calibration_store_init(void);

/**
 * @brief Carrega a calibracao da NVS e aplica no esc_pwm.
 *
 * Para cada motor, parte do valor default atual do esc_pwm e o sobrescreve com
 * o que houver na NVS (chaves ausentes mantem o default). Os valores aplicados
 * passam pela sanitizacao do esc_pwm.
 *
 * @code
 * (void)calibration_store_init();
 * calibration_store_load();   // antes de esc_pwm_init, no boot
 * @endcode
 */
void calibration_store_load(void);

/**
 * @brief Sanitiza a calibracao atual do esc_pwm e a grava na NVS.
 *
 * Chamar apos a interface alterar a calibracao, para torna-la permanente.
 */
void calibration_store_save(void);

/**
 * @brief Le os ganhos do PI vertical da NVS (mantem o default se ausentes).
 *
 * @param kp_us_per_ms    Saida: proporcional. Ignorado se NULL.
 * @param ki_us_per_ms    Saida: integral. Ignorado se NULL.
 * @param max_velocity_ms Saida: limite de velocidade. Ignorado se NULL.
 */
void vertical_params_load(float *kp_us_per_ms, float *ki_us_per_ms, float *max_velocity_ms);

/**
 * @brief Grava os ganhos do PI vertical na NVS (persiste a sintonia).
 */
void vertical_params_save(float kp_us_per_ms, float ki_us_per_ms, float max_velocity_ms);

#endif /* CALIBRATION_STORE_H */
