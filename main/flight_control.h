/**
 * @file flight_control.h
 * @brief Malha de controle de voo (cascata atitude->taxa) e logica de failsafe.
 *
 * Reune a tarefa de tempo real que estabiliza o drone e a maquina de estados de
 * seguranca. A cada ciclo (~20 ms) a tarefa: le a atitude da IMU, verifica as
 * condicoes de seguranca, calcula as correcoes pela cascata e escreve nos ESCs.
 *
 * Seguranca (failsafe): qualquer condicao perigosa (perda de comando, IMU
 * invalida/nao calibrada, inclinacao excessiva) desabilita o controle, leva os
 * motores ao minimo e "trava" (latch) o rearme ate o operador comandar PARAR
 * TUDO. Isso evita religamento automatico apos uma falha.
 *
 * Concorrencia: o estado de voo e protegido por secao critica (dados) e por
 * mutex (controladores), pois e acessado pela tarefa de controle e pelas rotas
 * do servidor web simultaneamente.
 *
 * Este e um modulo singleton: ::flight_control_init uma vez, depois
 * ::flight_control_start; as demais funcoes operam sobre o estado interno.
 */
#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "drone_pid.h"
#include "sensor_hub.h"
#include "vertical_control.h"

/**
 * @brief Comando completo vindo da interface (setpoints, estado manual e ganhos).
 *
 * Os campos manual_* so tem efeito quando o controle esta desabilitado (modo de
 * mixagem manual / preview); em voo estabilizado a atitude vem da IMU. Os
 * limites (saturacao) devem ser aplicados pelo chamador antes de preencher.
 */
typedef struct
{
    float throttle_us;        /**< Empuxo base (us). */
    float roll_setpoint_deg;  /**< Rolagem desejada. */
    float pitch_setpoint_deg; /**< Arfagem desejada. */
    float yaw_setpoint_deg;   /**< Guinada desejada. */
    float manual_roll_deg;    /**< Atitude manual (so com controle desabilitado). */
    float manual_pitch_deg;   /**< Atitude manual (so com controle desabilitado). */
    float manual_yaw_deg;     /**< Atitude manual (so com controle desabilitado). */
    pid_gains_t roll_gains;   /**< Ganhos de rolagem. */
    pid_gains_t pitch_gains;  /**< Ganhos de arfagem. */
    pid_gains_t yaw_gains;    /**< Ganhos de guinada. */
} flight_command_t;

/**
 * @brief Instantaneo do estado da malha, para telemetria/status.
 */
typedef struct
{
    flight_setpoint_t setpoint;            /**< Setpoint corrente. */
    flight_state_t state;                  /**< Atitude estimada corrente. */
    angular_rate_state_t rate;             /**< Velocidade angular medida. */
    angular_rate_setpoint_t rate_setpoint; /**< Setpoint de taxa da cascata. */
    axis_correction_t correction;          /**< Ultima correcao calculada. */
    quad_motor_output_t output;            /**< Ultima saida de motores. */
} flight_status_t;

/**
 * @brief Inicializa controladores, ganhos, mutexes e estado. Guarda o sensor_hub.
 *
 * Chamar uma vez, apos o sensor_hub iniciado e antes de ::flight_control_start.
 * @param hub Agregador de sensores de onde a malha le a atitude.
 */
void flight_control_init(sensor_hub_t *hub);

/**
 * @brief Cria a tarefa de controle de tempo real (core 1, prioridade alta).
 * @return true se a tarefa foi criada.
 */
bool flight_control_start(void);

/**
 * @brief Indica se a tarefa de controle existe.
 * @return true se criada (usado por /health e por /setFlight para recusar 503).
 */
bool flight_control_task_running(void);

/**
 * @brief Converte um motivo de failsafe em texto curto e estavel.
 * @param reason Motivo.
 * @return String constante (ex.: "COMMAND_TIMEOUT"); nunca NULL.
 */
const char *flight_failsafe_text(flight_failsafe_reason_t reason);

/** @brief Indica se a malha estabilizada esta ativa. */
bool flight_control_is_enabled(void);

/**
 * @brief Indica se ha failsafe travado aguardando PARAR TUDO.
 * @return true enquanto o rearme estiver bloqueado.
 */
bool flight_control_failsafe_latched(void);

/** @brief Motivo de failsafe corrente (para telemetria). */
flight_failsafe_reason_t flight_control_failsafe_reason(void);

/**
 * @brief Tempo (ms) desde o ultimo comando recebido.
 * @return Idade do comando; 0 se nenhum comando foi recebido ainda.
 */
uint32_t flight_control_command_age_ms(void);

/**
 * @brief Le os ganhos correntes dos tres eixos.
 * @param roll  Saida (ignorado se NULL).
 * @param pitch Saida (ignorado se NULL).
 * @param yaw   Saida (ignorado se NULL).
 */
void flight_control_get_gains(pid_gains_t *roll, pid_gains_t *pitch, pid_gains_t *yaw);

/**
 * @brief Copia um instantaneo consistente do estado da malha.
 * @param status Saida (ignorado se NULL).
 */
void flight_control_get_status(flight_status_t *status);

/**
 * @brief Aplica setpoints, estado manual e ganhos vindos da interface.
 *
 * Atualiza o setpoint sob secao critica e os ganhos dos controladores. Nao
 * habilita nem aplica nos motores por si so; combine com ::flight_control_mark_command
 * e, conforme o modo, ::flight_control_engage_stabilized ou ::flight_control_compute_mix.
 *
 * @param cmd Comando ja saturado pelo chamador (ignorado se NULL).
 */
void flight_control_apply_command(const flight_command_t *cmd);

/**
 * @brief Marca a chegada de um comando (reinicia o watchdog de timeout).
 *
 * Deve ser chamada a cada comando valido; se parar de ser chamada por mais que
 * o timeout, a malha entra em failsafe de perda de comando.
 */
void flight_control_mark_command(void);

/**
 * @brief Habilita a malha estabilizada (caminho aplicar+estabilizar aceito).
 *
 * Ao engatar a partir do estado desabilitado, alinha o setpoint de guinada ao
 * rumo atual da IMU e reinicia a cascata, para nao "puxar" o drone ao ligar.
 */
void flight_control_engage_stabilized(void);

/**
 * @brief Desabilita a malha, registrando o motivo; opcionalmente para os motores.
 *
 * Motivos perigosos travam o rearme (latch). Usada tanto pelos failsafes da
 * tarefa quanto pelas acoes do operador (override manual, parada de emergencia).
 *
 * @param reason      Motivo da desabilitacao.
 * @param stop_motors Se true, leva os motores ao minimo imediatamente.
 */
void flight_control_disable(flight_failsafe_reason_t reason, bool stop_motors);

/**
 * @brief Calcula a mixagem simples (PID angulo->us) e, opcionalmente, aplica.
 *
 * Modo manual/preview, sem realimentacao da IMU. Atualiza o instantaneo de
 * status e, se solicitado, escreve nos motores.
 *
 * @param apply_to_motors Se true, aplica a saida calculada aos ESCs.
 */
void flight_control_compute_mix(bool apply_to_motors);

/**
 * @brief Reinicia os PIDs (mixagem e cascata) e o relogio de mixagem.
 */
void flight_control_reset_pid(void);

/**
 * @brief Limpa o latch de failsafe, permitindo rearmar (apos PARAR TUDO).
 */
void flight_control_clear_failsafe_latch(void);

/* ===== Controle de velocidade vertical (funcionalidade adicional, sec. 11) =====
 * Desligado por padrao. Quando habilitado E com a malha estabilizada ativa, o
 * empuxo base passa a ser comandado pela malha PI de velocidade vertical (o stick
 * de throttle vira comando de velocidade). O estimador roda sempre, para
 * telemetria/validacao, sem atuar nos motores enquanto desengatado. */

/**
 * @brief Habilita/desabilita a malha de velocidade vertical (engate bumpless).
 * @param enable true para engatar; false para voltar ao empuxo direto.
 */
void flight_control_set_vertical_hold(bool enable);

/** @brief Indica se a malha de velocidade vertical esta habilitada. */
bool flight_control_vertical_hold_enabled(void);

/**
 * @brief Copia o estado do controle vertical (estimador + PI) para telemetria.
 * @param out Destino (ignorado se NULL).
 */
void flight_control_get_vertical(vertical_control_t *out);

/**
 * @brief Ajusta os ganhos do PI vertical em tempo real (para sintonia em voo).
 * @param kp_us_per_ms    Proporcional (us por m/s); ignorado se <= 0.
 * @param ki_us_per_ms    Integral (us por m/s por s); aceita 0, ignorado se < 0.
 * @param max_velocity_ms Limite do setpoint de velocidade (m/s); ignorado se <= 0.
 */
void flight_control_set_vertical_gains(float kp_us_per_ms, float ki_us_per_ms,
                                       float max_velocity_ms);

/**
 * @brief Liga/desliga a trava de seguranca do barometro do controle vertical.
 *
 * Com a trava ligada (padrao), o controle vertical so atua quando a referencia
 * do barometro esta pronta e recente; caso contrario desengata (devolve o
 * throttle ao operador). Desligar a trava permite operar so com o estimador
 * inercial (use apenas em teste controlado).
 */
void flight_control_set_vertical_baro_guard(bool enable);

/** @brief Indica se a trava do barometro do controle vertical esta ligada. */
bool flight_control_vertical_baro_guard_enabled(void);

#endif /* FLIGHT_CONTROL_H */
