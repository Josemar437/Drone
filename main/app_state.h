/**
 * @file app_state.h
 * @brief Flags globais minimas compartilhadas entre a inicializacao e o web server.
 *
 * Duas informacoes precisam atravessar fronteiras de modulo sem criar dependencia
 * circular entre app_main e web_server:
 *
 *  - `isArming`: enquanto os ESCs estao no periodo de arming, as rotas que movem
 *    motores devem responder 423 (Locked). app_main escreve; o web server le.
 *  - presenca da tarefa de telemetria: usada apenas pela rota /health para
 *    reportar saude do sistema.
 *
 * As flags sao volatile porque sao escritas por uma tarefa e lidas por outra.
 */
#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdbool.h>

/**
 * @brief Sinaliza se os ESCs ainda estao em arming.
 * @param arming true durante o arming; false quando o sistema esta operacional.
 */
void app_state_set_arming(bool arming);

/**
 * @brief Indica se o sistema ainda esta armando os ESCs.
 * @return true se em arming (rotas de motor devem recusar com 423).
 *
 * @code
 * if (app_state_is_arming()) {
 *     return send_text(req, "423 Locked", "Motores ainda armando");
 * }
 * @endcode
 */
bool app_state_is_arming(void);

/**
 * @brief Registra se a tarefa de telemetria foi criada com sucesso.
 * @param running true se a tarefa esta ativa.
 */
void app_state_set_telemetry_running(bool running);

/**
 * @brief Consulta a presenca da tarefa de telemetria (para /health).
 * @return true se a tarefa de telemetria esta ativa.
 */
bool app_state_telemetry_running(void);

#endif /* APP_STATE_H */
