/**
 * @file vertical_control.h
 * @brief Estimador de estado vertical + malha de velocidade vertical (climb-rate hold).
 *
 * Implementa a malha INTERNA do controle vertical em cascata: dado um setpoint de
 * velocidade vertical (m/s), gera o empuxo coletivo (us) a partir de um empuxo de
 * pairar (hover) somado a uma correcao PI sobre o erro de velocidade.
 *
 * A velocidade vertical NAO e derivada direto do barometro (ruidoso). Usa-se um
 * filtro complementar: a aceleracao vertical (IMU, projetada no eixo vertical do
 * mundo e com a gravidade removida) e integrada para velocidade/altitude, e a
 * altitude relativa do barometro corrige a deriva dessa integracao.
 *
 * Este modulo e logica pura (depende so de <math.h> e das constantes do projeto),
 * para poder ser validado isoladamente. Fica desligado por padrao e separado do
 * caminho de controle de atitude congelado.
 *
 * @warning A projecao/remocao da gravidade depende de roll/pitch corretos. Se a
 * IMU estiver mal orientada (atitude errada com o drone nivelado), o estimador
 * vertical divergira. Validar a orientacao da IMU antes de habilitar.
 */
#ifndef VERTICAL_CONTROL_H
#define VERTICAL_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Estado do estimador vertical e da malha PI de velocidade.
 */
typedef struct
{
    /* Estimador */
    float altitude_est_m;     /**< Altitude relativa estimada (m). */
    float velocity_est_ms;    /**< Velocidade vertical estimada (m/s, + = subindo). */
    float accel_vertical_ms2; /**< Ultima aceleracao vertical (m/s^2, gravidade removida). */
    bool initialized;         /**< true apos a 1a amostra (semente da altitude). */
    /* Controlador PI (parametros tunaveis em runtime) */
    float kp_us_per_ms;       /**< Ganho proporcional (us por m/s). */
    float ki_us_per_ms;       /**< Ganho integral (us por m/s por s). */
    float max_velocity_ms;    /**< Saturacao do setpoint de velocidade (m/s). */
    float integral_us;        /**< Termo integral (us), com anti-windup. */
    float hover_throttle_us;  /**< Empuxo de pairar usado como base. */
    float vz_setpoint_ms;     /**< Ultimo setpoint de velocidade aplicado (m/s). */
    float throttle_output_us; /**< Ultimo empuxo coletivo calculado (us). */
    bool engaged;             /**< true quando a malha esta atuando no empuxo. */
    bool saturated;           /**< true se a saida saturou (para diagnostico/anti-windup). */
} vertical_control_t;

/**
 * @brief Inicializa o estado (zera estimador e PI; hover = default de config).
 */
void vertical_control_init(vertical_control_t *vc);

/**
 * @brief Atualiza o estimador complementar de altitude/velocidade vertical.
 *
 * Deve ser chamado a cada ciclo (mesmo com a malha desengatada) para manter a
 * estimativa pronta e permitir validacao via telemetria sem atuar nos motores.
 *
 * @param vc          Instancia.
 * @param ax_g        Aceleracao do corpo X (g).
 * @param ay_g        Aceleracao do corpo Y (g).
 * @param az_g        Aceleracao do corpo Z (g).
 * @param roll_deg    Rolagem estimada (graus).
 * @param pitch_deg   Arfagem estimada (graus).
 * @param baro_alt_m  Altitude relativa do barometro (m).
 * @param baro_valid  true se baro_alt_m e confiavel (corrige a deriva).
 * @param dt_seconds  Intervalo desde o ultimo passo (s). <= 0 e ignorado.
 */
void vertical_estimator_update(vertical_control_t *vc, float ax_g, float ay_g, float az_g,
                               float roll_deg, float pitch_deg, float baro_alt_m,
                               bool baro_valid, float dt_seconds);

/**
 * @brief Engata a malha de forma "sem salto" (bumpless).
 *
 * Zera o integral e adota o empuxo aplicado no momento como hover, para que a
 * saida inicial coincida com o empuxo corrente (sem degrau no motor).
 *
 * @param vc                   Instancia.
 * @param current_throttle_us  Empuxo aplicado no instante do engate (us).
 */
void vertical_control_engage(vertical_control_t *vc, float current_throttle_us);

/** @brief Desengata a malha (volta ao empuxo direto do operador). */
void vertical_control_disengage(vertical_control_t *vc);

/**
 * @brief Ajusta os ganhos do PI e o limite de velocidade em tempo real.
 *
 * Permite sintonizar durante os testes sem recompilar. Valores nao positivos
 * sao ignorados (mantem o anterior), exceto ki que pode ser 0.
 *
 * @param vc              Instancia.
 * @param kp_us_per_ms    Ganho proporcional (us por m/s). Ignorado se <= 0.
 * @param ki_us_per_ms    Ganho integral (us por m/s por s). Aceita 0; ignorado se < 0.
 * @param max_velocity_ms Saturacao do setpoint de velocidade (m/s). Ignorado se <= 0.
 */
void vertical_control_set_gains(vertical_control_t *vc, float kp_us_per_ms,
                                float ki_us_per_ms, float max_velocity_ms);

/**
 * @brief Calcula o empuxo coletivo (us) para um setpoint de velocidade vertical.
 *
 * throttle = hover + PI(vz_setpoint - vz_estimada), com compensacao de inclinacao
 * (throttle/(cosR*cosP), limitada) e saturacao. O integral so acumula quando a
 * saida nao esta saturada (anti-windup condicionado a saturacao real, incluindo
 * a do mixer).
 *
 * @param vc              Instancia (deve estar inicializada/estimando).
 * @param vz_setpoint_ms  Velocidade vertical desejada (m/s); sera saturada.
 * @param roll_deg        Rolagem atual (graus) para a compensacao de inclinacao.
 * @param pitch_deg       Arfagem atual (graus) para a compensacao de inclinacao.
 * @param dt_seconds      Intervalo do passo (s).
 * @param mixer_saturated true se a mixagem de atitude ja saturou algum motor.
 * @return Empuxo coletivo (us) saturado a [ESC_MIN_US, ESC_MAX_US].
 */
float vertical_velocity_hold(vertical_control_t *vc, float vz_setpoint_ms,
                             float roll_deg, float pitch_deg, float dt_seconds,
                             bool mixer_saturated);

/**
 * @brief Mapeia o stick de throttle (us) em setpoint de velocidade vertical (m/s).
 *
 * Centro (faixa morta) = manter 0 m/s; acima sobe ate +VERT_MAX_VELOCITY_MS,
 * abaixo desce ate -VERT_MAX_VELOCITY_MS.
 *
 * @param throttle_us     Posicao do stick em us (1000..2000).
 * @param max_velocity_ms Velocidade maxima nas extremidades do stick (m/s).
 * @return Velocidade vertical desejada (m/s).
 */
float vertical_throttle_to_setpoint(float throttle_us, float max_velocity_ms);

#endif /* VERTICAL_CONTROL_H */
