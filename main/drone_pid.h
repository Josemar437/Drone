/**
 * @file drone_pid.h
 * @brief Controladores PID e mixagem de motores em configuracao X.
 *
 * Tres camadas, da mais simples a mais completa:
 *
 *  1. ::pid_controller_t  - um PID escalar generico (um eixo).
 *  2. ::quad_pid_controller_t - tres PIDs (roll/pitch/yaw) que convertem erro de
 *     angulo diretamente em correcao de microssegundos, mais a mixagem X.
 *     Usado no modo "mixagem manual" (sem realimentacao da IMU).
 *  3. ::attitude_rate_controller_t - controle em cascata: um termo P externo
 *     transforma erro de angulo em setpoint de velocidade angular, e um PID
 *     interno por eixo regula essa velocidade. E o modo usado em voo
 *     estabilizado, por rejeitar perturbacoes melhor que o PID de angulo puro.
 *
 * As formulas e a ordem das operacoes em ponto flutuante sao definidas com
 * cuidado para garantir saida deterministica.
 *
 * Como sao "objetos" em C, cada funcao recebe um ponteiro para o estado
 * (`self`/`quad`/`ctrl`) que deve ter sido inicializado pela respectiva
 * funcao *_init antes do uso.
 */
#ifndef DRONE_PID_H
#define DRONE_PID_H

#include <stdint.h>
#include <stdbool.h>

/** @brief Ganhos de um PID: proporcional, integral e derivativo. */
typedef struct
{
    float kp; /**< Ganho proporcional. */
    float ki; /**< Ganho integral. */
    float kd; /**< Ganho derivativo. */
} pid_gains_t;

/** @brief Setpoint de voo: empuxo base (us) e angulos-alvo (graus). */
typedef struct
{
    float throttle_us; /**< Empuxo base aplicado igualmente aos 4 motores. */
    float roll_deg;    /**< Angulo de rolagem desejado. */
    float pitch_deg;   /**< Angulo de arfagem desejado. */
    float yaw_deg;     /**< Rumo (heading) desejado. */
} flight_setpoint_t;

/** @brief Atitude atual estimada do drone (graus). */
typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} flight_state_t;

/** @brief Correcao por eixo a aplicar na mixagem (em us de pulso). */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} axis_correction_t;

/** @brief Saida de pulso (us) para cada um dos 4 motores. */
typedef struct
{
    int32_t m1; /**< Frente-esquerda. */
    int32_t m2; /**< Frente-direita. */
    int32_t m3; /**< Traseira-direita. */
    int32_t m4; /**< Traseira-esquerda. */
} quad_motor_output_t;

/** @brief Resultado de mixagem com diagnostico de saturacao. */
typedef struct
{
    quad_motor_output_t output;     /**< Pulsos finais, ja saturados. */
    bool saturated;                 /**< true se algum motor exigiu desaturacao/clamp. */
    float collective_adjust_us;     /**< Deslocamento comum aplicado ao coletivo. */
} quad_mix_result_t;

/** @brief Velocidade angular medida pelo giroscopio (graus/s). */
typedef struct
{
    float roll_dps;
    float pitch_dps;
    float yaw_dps;
} angular_rate_state_t;

/** @brief Setpoint de velocidade angular gerado pela malha externa (graus/s). */
typedef struct
{
    float roll_dps;
    float pitch_dps;
    float yaw_dps;
} angular_rate_setpoint_t;

/**
 * @brief Motivo pelo qual o controle de voo foi (ou nao) desabilitado.
 *
 * Reportado na telemetria e usado para travar (latch) o rearme apos uma
 * condicao perigosa, exigindo intervencao explicita do operador.
 */
typedef enum
{
    FLIGHT_FAILSAFE_NONE = 0,             /**< Operacao normal. */
    FLIGHT_FAILSAFE_COMMAND_TIMEOUT,      /**< Perda de link com a interface. */
    FLIGHT_FAILSAFE_MPU_INVALID,          /**< Leitura da IMU invalida/antiga. */
    FLIGHT_FAILSAFE_MPU_NOT_CALIBRATED,   /**< IMU ainda nao calibrada. */
    FLIGHT_FAILSAFE_EXCESSIVE_TILT,       /**< Inclinacao acima do seguro. */
    FLIGHT_FAILSAFE_MANUAL_OVERRIDE,      /**< Comando manual assumiu o controle. */
    FLIGHT_FAILSAFE_EMERGENCY_STOP,       /**< Parada de emergencia (operador). */
    FLIGHT_FAILSAFE_CONTROLLER_TIMEOUT    /**< Timeout do mutex da cascata (compute nao concluiu). */
} flight_failsafe_reason_t;

/**
 * @brief PID escalar com anti-windup e saturacao de saida.
 *
 * Mantem o estado entre chamadas (integral e erro anterior). Os limites de
 * integral e de saida evitam, respectivamente, acumulo excessivo (windup) e
 * comandos fora da faixa fisica do atuador.
 */
typedef struct
{
    pid_gains_t gains;
    float integral;
    float previous_error;
    float output_min;
    float output_max;
    float integral_limit;
    bool has_previous_error; /**< false ate o 1o update (derivada parte de 0). */
} pid_controller_t;

/**
 * @brief Inicializa um PID com ganhos zerados e limites default
 *        (saida +/-250, integral +/-150).
 * @param pid Instancia a inicializar. Chame antes de qualquer outra funcao.
 */
void pid_init(pid_controller_t *pid);

/** @brief Define os ganhos kp/ki/kd. */
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/** @brief Define a saturacao da saida do PID. */
void pid_set_output_limits(pid_controller_t *pid, float min_output, float max_output);

/**
 * @brief Define o limite (simetrico) do termo integral.
 * @param pid   Instancia do PID.
 * @param limit Valor maximo absoluto; o sinal e ignorado (usa-se |limit|).
 */
void pid_set_integral_limit(pid_controller_t *pid, float limit);

/**
 * @brief Zera o estado interno (integral e historico de erro).
 *
 * Chamar ao (re)habilitar o controle evita que um integral antigo cause um
 * "salto" no primeiro comando.
 */
void pid_reset(pid_controller_t *pid);

/**
 * @brief Executa um passo do PID.
 *
 * @param pid          Instancia.
 * @param setpoint     Valor desejado.
 * @param measurement  Valor medido.
 * @param dt_seconds   Intervalo desde o ultimo passo, em segundos.
 * @return Saida saturada. Retorna 0 se dt_seconds <= 0 (passo invalido).
 *
 * @code
 * float u = pid_update(&pid, alvo, medido, 0.02f); // passo de 20 ms
 * @endcode
 */
float pid_update(pid_controller_t *pid, float setpoint, float measurement,
                 float dt_seconds);

/**
 * @brief Conjunto de tres PIDs (roll/pitch/yaw) angulo->us + mixagem X.
 *
 * Modo direto: o erro de angulo vira correcao em microssegundos, somada ao
 * empuxo base na mixagem. Mais simples que a cascata, util sem IMU (preview de
 * mixagem) ou como base de comparacao.
 */
typedef struct
{
    pid_controller_t roll_pid;
    pid_controller_t pitch_pid;
    pid_controller_t yaw_pid;
} quad_pid_controller_t;

/**
 * @brief Inicializa os tres PIDs com ganhos default e limites de voo
 *        (saida +/-250 us, integral +/-120).
 */
void quad_pid_init(quad_pid_controller_t *quad);

/** @brief Ajusta os ganhos do eixo de rolagem. */
void quad_pid_set_roll_gains(quad_pid_controller_t *quad, float kp, float ki, float kd);
/** @brief Ajusta os ganhos do eixo de arfagem. */
void quad_pid_set_pitch_gains(quad_pid_controller_t *quad, float kp, float ki, float kd);
/** @brief Ajusta os ganhos do eixo de guinada. */
void quad_pid_set_yaw_gains(quad_pid_controller_t *quad, float kp, float ki, float kd);
/** @brief Define a saturacao de saida (us) igual para os tres eixos. */
void quad_pid_set_output_limits(quad_pid_controller_t *quad, float min_us, float max_us);
/** @brief Define o limite de integral igual para os tres eixos. */
void quad_pid_set_integral_limit(quad_pid_controller_t *quad, float limit);
/** @brief Zera os tres PIDs (use ao reabilitar o controle). */
void quad_pid_reset(quad_pid_controller_t *quad);

/**
 * @brief Calcula a correcao dos tres eixos a partir de setpoint e estado.
 * @param quad       Instancia.
 * @param setpoint   Angulos desejados.
 * @param state      Atitude atual.
 * @param dt_seconds Intervalo do passo (s).
 * @return Correcao por eixo (em us), pronta para ::quad_pid_mix_x.
 */
axis_correction_t quad_pid_compute(quad_pid_controller_t *quad,
                                   const flight_setpoint_t *setpoint,
                                   const flight_state_t *state, float dt_seconds);

/**
 * @brief Mistura empuxo + correcoes na saida dos 4 motores (config X).
 *
 * Cada motor recebe o empuxo base mais/menos as correcoes conforme sua posicao:
 * arfagem afeta frente vs tras, rolagem afeta esquerda vs direita, guinada afeta
 * os pares de rotacao oposta. O resultado e saturado a [esc_min, esc_max].
 *
 * @param throttle_us Empuxo base aplicado aos quatro motores.
 * @param correction  Correcoes por eixo (de ::quad_pid_compute ou da cascata).
 * @param esc_min     Pulso minimo (saturacao inferior).
 * @param esc_max     Pulso maximo (saturacao superior).
 * @return Pulsos por motor, ja saturados.
 *
 * @code
 * quad_motor_output_t m = quad_pid_mix_x(setpoint.throttle_us, &corr, 1000, 2000);
 * @endcode
 */
quad_motor_output_t quad_pid_mix_x(float throttle_us, const axis_correction_t *correction,
                                   int32_t esc_min, int32_t esc_max);

/**
 * @brief Mixagem X com desaturacao coletiva.
 *
 * Antes do clamp final, desloca todos os motores pelo mesmo valor para manter
 * as diferencas relativas de roll/pitch/yaw quando o coletivo empurra um motor
 * para fora da faixa fisica. Isso preserva autoridade de atitude melhor que
 * truncar cada motor isoladamente, principalmente em decolagem e hover.
 *
 * @return Saida final e diagnostico de saturacao para telemetria/anti-windup.
 */
quad_mix_result_t quad_pid_mix_x_desaturated(float throttle_us,
                                             const axis_correction_t *correction,
                                             int32_t esc_min, int32_t esc_max);

/**
 * @brief Controle em cascata atitude->velocidade angular (modo estabilizado).
 *
 * Malha externa (P): erro de angulo * Kp -> setpoint de velocidade angular,
 * saturado a limites seguros. Malha interna (PID por eixo): regula essa
 * velocidade usando o giroscopio. A cascata responde melhor a rajadas/choques
 * porque atua sobre a taxa medida, nao apenas sobre o angulo.
 */
typedef struct
{
    pid_controller_t roll_rate_pid;
    pid_controller_t pitch_rate_pid;
    pid_controller_t yaw_rate_pid;
    angular_rate_setpoint_t rate_setpoint; /**< Ultimo setpoint de taxa calculado. */
    float roll_attitude_kp;                /**< Kp da malha externa (rolagem). */
    float pitch_attitude_kp;               /**< Kp da malha externa (arfagem). */
    float yaw_attitude_kp;                 /**< Kp da malha externa (guinada). */
} attitude_rate_controller_t;

/**
 * @brief Inicializa a cascata com ganhos default (externo e interno) e limites.
 */
void attitude_rate_init(attitude_rate_controller_t *ctrl);

/**
 * @brief Zera os PIDs internos e o setpoint de taxa.
 *
 * Chamar ao engatar a estabilizacao para comecar de um estado limpo.
 */
void attitude_rate_reset(attitude_rate_controller_t *ctrl);

/**
 * @brief Ajusta os Kp da malha externa (atitude) dos tres eixos.
 *
 * Permite que a interface reaproveite os mesmos campos Kp dos ganhos de voo
 * para sintonizar a resposta de atitude.
 */
void attitude_rate_set_attitude_gains(attitude_rate_controller_t *ctrl,
                                      float roll_kp, float pitch_kp, float yaw_kp);

/** @brief Define a saturacao de saida (us) dos tres PIDs internos. */
void attitude_rate_set_output_limits(attitude_rate_controller_t *ctrl,
                                     float min_us, float max_us);

/**
 * @brief Executa um passo da cascata.
 *
 * @param ctrl       Instancia.
 * @param setpoint   Angulos desejados (usa roll/pitch/yaw).
 * @param attitude   Atitude atual estimada.
 * @param rate       Velocidade angular medida pelo giroscopio.
 * @param dt_seconds Intervalo do passo (s).
 * @return Correcao por eixo (us) para a mixagem.
 *
 * @code
 * axis_correction_t c = attitude_rate_compute(&ctrl, &sp, &att, &gyro, 0.02f);
 * quad_motor_output_t m = quad_pid_mix_x(sp.throttle_us, &c, 1000, 2000);
 * @endcode
 */
axis_correction_t attitude_rate_compute(attitude_rate_controller_t *ctrl,
                                        const flight_setpoint_t *setpoint,
                                        const flight_state_t *attitude,
                                        const angular_rate_state_t *rate,
                                        float dt_seconds);

/**
 * @brief Executa a cascata com yaw em modo taxa angular direta.
 *
 * Roll/pitch continuam em malha externa de atitude. O yaw recebe diretamente o
 * setpoint de velocidade angular ja limitado/suavizado pela logica de voo.
 */
axis_correction_t attitude_rate_compute_yaw_rate(attitude_rate_controller_t *ctrl,
                                                const flight_setpoint_t *setpoint,
                                                const flight_state_t *attitude,
                                                const angular_rate_state_t *rate,
                                                float yaw_rate_setpoint_dps,
                                                float dt_seconds);

/**
 * @brief Retorna o ultimo setpoint de velocidade angular calculado.
 *
 * Util para telemetria/diagnostico (mostra o alvo da malha interna).
 */
angular_rate_setpoint_t attitude_rate_setpoint(const attitude_rate_controller_t *ctrl);

#endif /* DRONE_PID_H */
