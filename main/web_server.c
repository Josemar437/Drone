/**
 * @file web_server.c
 * @brief Implementacao do servidor HTTP e das rotas (ver web_server.h).
 *
 * Cada rota e um handler do esp_http_server. As respostas (texto/JSON) e os
 * codigos de status sao consumidos pela pagina web embutida no binario
 * (EMBED_TXTFILES index.html), servida pela rota raiz.
 *
 * Desvio MISRA (Rule 21.6 / Dir 4.9): o JSON e montado com snprintf via a macro
 * delimitada J_APPEND. E a forma segura (limitada por tamanho) e legivel; nao faz
 * parte da cadeia de PWM.
 */
#include "web_server.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "app_math.h"
#include "app_time.h"
#include "app_state.h"
#include "esc_pwm.h"
#include "flight_control.h"
#include "calibration_store.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "web";

/* Pagina HTML embutida no binario (null-terminada por EMBED_TXTFILES). O simbolo
 * _binary_index_html_start e gerado pelo build a partir de main/index.html. */
extern const char index_html_start[] __asm__("_binary_index_html_start");

static httpd_handle_t s_server = NULL;
static sensor_hub_t *s_hub = NULL;

/**
 * @brief Acrescenta texto formatado a um buffer JSON, sem estourar a capacidade.
 *
 * Requer as variaveis locais 'buf' (destino), 'off' (offset corrente) e 'cap'
 * (capacidade) no escopo. Atualiza 'off' e nunca escreve alem de 'cap'.
 */
#define J_APPEND(...)                                                      \
    do {                                                                   \
        if (off < cap) {                                                   \
            const int n = snprintf(&buf[off], cap - off, __VA_ARGS__);     \
            if (n > 0) {                                                   \
                off += (size_t)n;                                          \
                if (off > cap) { off = cap; }                             \
            }                                                              \
        }                                                                  \
    } while (0)

/** @brief Converte um booleano no literal JSON correspondente. */
static const char *bool_str(bool value)
{
    return value ? "true" : "false";
}

/** @brief Converte um pulso logico (us) no percentual 0..100 exibido na interface. */
static int32_t speed_to_pct(int32_t speed_us)
{
    return ((speed_us - ESC_MIN_US) * 100) / (ESC_MAX_US - ESC_MIN_US);
}

/* ===== Helpers de resposta ===== */

/**
 * @brief Responde texto puro com um status HTTP especifico.
 * @param status Linha de status (ex.: "423 Locked").
 */
static esp_err_t send_text(httpd_req_t *req, const char *status, const char *body)
{
    (void)httpd_resp_set_status(req, status);
    (void)httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, body);
}

/** @brief Responde um corpo JSON com status 200. */
static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    (void)httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

/* ===== Helpers de query =====
 * Encapsulam a leitura da query string e a extracao de parametros, usando o
 * valor corrente como default quando o campo falta. */

/**
 * @brief Copia a query string da requisicao para um buffer.
 * @return true se havia query e coube no buffer; false caso contrario.
 */
static bool read_query(httpd_req_t *req, char *buf, size_t len)
{
    const size_t query_len = httpd_req_get_url_query_len(req) + 1U;
    buf[0] = '\0';
    if ((query_len <= 1U) || (query_len > len))
    {
        return false;
    }
    return (httpd_req_get_url_query_str(req, buf, len) == ESP_OK);
}

/** @brief Le o valor textual de um parametro da query. */
static bool arg_str(const char *query, const char *key, char *value, size_t value_len)
{
    return (httpd_query_key_value(query, key, value, value_len) == ESP_OK);
}

/** @brief Indica se um parametro esta presente na query (equivale a hasArg). */
static bool arg_present(const char *query, const char *key)
{
    char value[16];
    return arg_str(query, key, value, sizeof(value));
}

/** @brief Le um parametro inteiro; retorna 'fallback' se ausente. */
static int32_t arg_int(const char *query, const char *key, int32_t fallback)
{
    char value[24];
    int32_t result = fallback;
    if (arg_str(query, key, value, sizeof(value)))
    {
        result = (int32_t)strtol(value, NULL, 10);
    }
    return result;
}

/** @brief Le um parametro em ponto flutuante; retorna 'fallback' se ausente. */
static float arg_float(const char *query, const char *key, float fallback)
{
    char value[32];
    float result = fallback;
    if (arg_str(query, key, value, sizeof(value)))
    {
        result = strtof(value, NULL);
    }
    return result;
}

/** @brief Indica se um parametro esta presente e igual ao valor esperado. */
static bool arg_equals(const char *query, const char *key, const char *expected)
{
    char value[8];
    return arg_str(query, key, value, sizeof(value)) && (strcmp(value, expected) == 0);
}

/* ===== Rotas ===== */

/** @brief GET / : serve a pagina de controle (HTML embutido), sem cache. */
static esp_err_t root_handler(httpd_req_t *req)
{
    (void)httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    (void)httpd_resp_set_hdr(req, "Pragma", "no-cache");
    (void)httpd_resp_set_hdr(req, "Expires", "0");
    (void)httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_sendstr(req, index_html_start);
}

/**
 * @brief GET /setMotor?motor=&speed= : aciona um motor por percentual (0..100).
 *
 * Assume override manual (desabilita a malha estabilizada). Recusa com 423
 * enquanto em arming. Util em teste de motor individual na bancada.
 */
static esp_err_t set_motor_handler(httpd_req_t *req)
{
    char query[64];
    if (app_state_is_arming())
    {
        return send_text(req, "423 Locked", "Motores ainda armando");
    }
    if (read_query(req, query, sizeof(query)) && arg_present(query, "motor") &&
        arg_present(query, "speed"))
    {
        const int32_t motor = arg_int(query, "motor", 0);
        const int32_t percentage = clamp_i32(arg_int(query, "speed", 0), 0, 100);
        const int32_t speed = ESC_MIN_US + ((percentage * (ESC_MAX_US - ESC_MIN_US)) / 100);
        flight_control_disable(FLIGHT_FAILSAFE_MANUAL_OVERRIDE, flight_control_is_enabled());
        if ((motor >= 0) && (motor < NUM_MOTORS))
        {
            esc_set_motor_speed((uint8_t)motor, speed);
        }
        return send_text(req, "200 OK", "OK");
    }
    return send_text(req, "400 Bad Request", "Parametros faltando");
}

/**
 * @brief GET /stopAll : parada de emergencia.
 *
 * Desabilita o controle, leva os motores ao minimo e limpa o latch de failsafe,
 * permitindo rearmar em seguida. E a acao que o operador usa para destravar o
 * sistema apos qualquer failsafe.
 */
static esp_err_t stop_all_handler(httpd_req_t *req)
{
    if (app_state_is_arming())
    {
        return send_text(req, "423 Locked", "Motores ainda armando");
    }
    flight_control_disable(FLIGHT_FAILSAFE_EMERGENCY_STOP, false);
    esc_stop_all_motors();
    flight_control_clear_failsafe_latch();
    return send_text(req, "200 OK", "Todos parados");
}

/** @brief GET /status : velocidades/saidas dos motores e estado do controle (JSON). */
static esp_err_t status_handler(httpd_req_t *req)
{
    char buf[320];
    size_t off = 0U;
    const size_t cap = sizeof(buf);
    uint8_t i;

    J_APPEND("{\"speeds\":[");
    for (i = 0U; i < (uint8_t)NUM_MOTORS; i++)
    {
        J_APPEND("%d%s", (int)speed_to_pct(esc_get_motor_speed(i)),
                 (i < (NUM_MOTORS - 1)) ? "," : "");
    }
    J_APPEND("],\"outputs\":[");
    for (i = 0U; i < (uint8_t)NUM_MOTORS; i++)
    {
        J_APPEND("%d%s", (int)esc_get_motor_output_us(i), (i < (NUM_MOTORS - 1)) ? "," : "");
    }
    J_APPEND("],\"arming\":%s,\"flightControlEnabled\":%s,\"failsafe\":\"%s\"}",
             bool_str(app_state_is_arming()), bool_str(flight_control_is_enabled()),
             flight_failsafe_text(flight_control_failsafe_reason()));
    return send_json(req, buf);
}

/**
 * @brief GET /sensors : telemetria completa dos sensores (JSON).
 *
 * Usa buffer estatico (grande) pois o httpd atende uma requisicao por vez nesta
 * tarefa; evita alocacao dinamica e estouro de pilha.
 */
static esp_err_t sensors_handler(httpd_req_t *req)
{
    static char json[SENSOR_HUB_JSON_CAPACITY];
    (void)httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    (void)sensor_hub_to_json(s_hub, json, sizeof(json));
    return send_json(req, json);
}

/**
 * @brief GET /health : diagnostico de sistema (heap, tarefas, motores) em JSON.
 *
 * Usado para verificar rapidamente, sem osciloscopio, se o firmware subiu
 * integro: memoria livre, presenca das tarefas e se os motores estao no minimo.
 */
static esp_err_t health_handler(httpd_req_t *req)
{
    char buf[256];
    size_t off = 0U;
    const size_t cap = sizeof(buf);
    bool motors_at_minimum = true;
    uint8_t motor;

    for (motor = 0U; motor < (uint8_t)NUM_MOTORS; motor++)
    {
        if (esc_get_motor_speed(motor) != ESC_MIN_US)
        {
            motors_at_minimum = false;
        }
    }

    J_APPEND("{\"ok\":true,\"heap\":%u,\"maxAllocHeap\":%u,\"htmlBytes\":%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
             (unsigned)strlen(index_html_start));
    J_APPEND(",\"commandTask\":%s,\"telemetryTask\":%s,\"flightTask\":%s",
             bool_str(web_server_running()), bool_str(app_state_telemetry_running()),
             bool_str(flight_control_task_running()));
    J_APPEND(",\"motorsAtMinimum\":%s}", bool_str(motors_at_minimum));
    return send_json(req, buf);
}

/**
 * @brief Monta e envia o JSON de calibracao (start/max/trim/saidas) dos motores.
 *
 * Reutilizada pelas tres rotas de calibracao para responder o estado atualizado.
 */
static esp_err_t send_calibration(httpd_req_t *req)
{
    char buf[384];
    size_t off = 0U;
    const size_t cap = sizeof(buf);
    int32_t start_us;
    int32_t max_us;
    int32_t trim_us;
    uint8_t i;

    J_APPEND("{\"start\":[");
    for (i = 0U; i < (uint8_t)NUM_MOTORS; i++)
    {
        esc_get_calibration(i, &start_us, &max_us, &trim_us);
        J_APPEND("%d%s", (int)start_us, (i < (NUM_MOTORS - 1)) ? "," : "");
    }
    J_APPEND("],\"max\":[");
    for (i = 0U; i < (uint8_t)NUM_MOTORS; i++)
    {
        esc_get_calibration(i, &start_us, &max_us, &trim_us);
        J_APPEND("%d%s", (int)max_us, (i < (NUM_MOTORS - 1)) ? "," : "");
    }
    J_APPEND("],\"trim\":[");
    for (i = 0U; i < (uint8_t)NUM_MOTORS; i++)
    {
        esc_get_calibration(i, &start_us, &max_us, &trim_us);
        J_APPEND("%d%s", (int)trim_us, (i < (NUM_MOTORS - 1)) ? "," : "");
    }
    J_APPEND("],\"outputs\":[");
    for (i = 0U; i < (uint8_t)NUM_MOTORS; i++)
    {
        J_APPEND("%d%s", (int)esc_get_motor_output_us(i), (i < (NUM_MOTORS - 1)) ? "," : "");
    }
    J_APPEND("],\"arming\":%s}", bool_str(app_state_is_arming()));
    return send_json(req, buf);
}

/** @brief GET /calibration : le a calibracao atual dos motores (JSON). */
static esp_err_t calibration_handler(httpd_req_t *req)
{
    return send_calibration(req);
}

/**
 * @brief GET /setCalibration?startN=&maxN=&trimN= : grava a calibracao por motor.
 *
 * Aplica os valores informados (mantendo os atuais nos campos omitidos), persiste
 * na NVS, reaplica as saidas e responde a calibracao resultante. Desabilita a
 * malha (override manual) por seguranca antes de mexer na calibracao.
 */
static esp_err_t set_calibration_handler(httpd_req_t *req)
{
    char query[256];
    uint8_t i;

    if (app_state_is_arming())
    {
        return send_text(req, "423 Locked", "Motores ainda armando");
    }
    flight_control_disable(FLIGHT_FAILSAFE_MANUAL_OVERRIDE, true);
    (void)read_query(req, query, sizeof(query));

    for (i = 0U; i < (uint8_t)NUM_MOTORS; i++)
    {
        char key[12];
        int32_t start_us;
        int32_t max_us;
        int32_t trim_us;
        esc_get_calibration(i, &start_us, &max_us, &trim_us);
        (void)snprintf(key, sizeof(key), "start%u", (unsigned)i);
        start_us = arg_int(query, key, start_us);
        (void)snprintf(key, sizeof(key), "max%u", (unsigned)i);
        max_us = arg_int(query, key, max_us);
        (void)snprintf(key, sizeof(key), "trim%u", (unsigned)i);
        trim_us = arg_int(query, key, trim_us);
        esc_set_calibration(i, start_us, max_us, trim_us);
    }

    calibration_store_save();
    esc_reapply_current_outputs();
    return send_calibration(req);
}

/** @brief GET /resetCalibration : restaura defaults de fabrica, persiste e reaplica. */
static esp_err_t reset_calibration_handler(httpd_req_t *req)
{
    if (app_state_is_arming())
    {
        return send_text(req, "423 Locked", "Motores ainda armando");
    }
    flight_control_disable(FLIGHT_FAILSAFE_MANUAL_OVERRIDE, true);
    esc_reset_calibration_to_defaults();
    calibration_store_save();
    esc_reapply_current_outputs();
    return send_calibration(req);
}

/**
 * @brief GET /setFlight?... : recebe setpoints/ganhos e comanda a malha.
 *
 * E a rota central de voo. Le e satura os parametros (usando os valores atuais
 * como default), aplica o comando e marca o watchdog. Dois caminhos:
 *  - apply=1 & stabilize=1: engata a malha estabilizada (recusa 503 se a tarefa
 *    nao existe, 409 se ha failsafe travado);
 *  - caso contrario: faz a mixagem manual e, se apply=1, aplica aos motores.
 *
 * O envio deve ser repetido (< timeout) para manter a malha ativa.
 */
static esp_err_t set_flight_handler(httpd_req_t *req)
{
    char query[512];
    flight_status_t st;
    pid_gains_t roll_gains;
    pid_gains_t pitch_gains;
    pid_gains_t yaw_gains;
    flight_command_t cmd;
    bool apply;
    bool stabilize;

    if (app_state_is_arming())
    {
        return send_text(req, "423 Locked", "Motores ainda armando");
    }
    (void)read_query(req, query, sizeof(query));
    flight_control_get_status(&st);
    flight_control_get_gains(&roll_gains, &pitch_gains, &yaw_gains);

    cmd.throttle_us = clamp_f(arg_float(query, "throttle", st.setpoint.throttle_us),
                              (float)ESC_MIN_US, (float)ESC_MAX_US);
    cmd.roll_setpoint_deg = clamp_f(arg_float(query, "rollSp", st.setpoint.roll_deg), -35.0f, 35.0f);
    cmd.pitch_setpoint_deg = clamp_f(arg_float(query, "pitchSp", st.setpoint.pitch_deg), -35.0f, 35.0f);
    cmd.yaw_setpoint_deg = clamp_f(arg_float(query, "yawSp", st.setpoint.yaw_deg), -180.0f, 180.0f);
    cmd.manual_roll_deg = clamp_f(arg_float(query, "rollState", st.state.roll_deg), -180.0f, 180.0f);
    cmd.manual_pitch_deg = clamp_f(arg_float(query, "pitchState", st.state.pitch_deg), -180.0f, 180.0f);
    cmd.manual_yaw_deg = clamp_f(arg_float(query, "yawState", st.state.yaw_deg), -180.0f, 180.0f);
    cmd.roll_gains.kp = arg_float(query, "rollKp", roll_gains.kp);
    cmd.roll_gains.ki = arg_float(query, "rollKi", roll_gains.ki);
    cmd.roll_gains.kd = arg_float(query, "rollKd", roll_gains.kd);
    cmd.pitch_gains.kp = arg_float(query, "pitchKp", pitch_gains.kp);
    cmd.pitch_gains.ki = arg_float(query, "pitchKi", pitch_gains.ki);
    cmd.pitch_gains.kd = arg_float(query, "pitchKd", pitch_gains.kd);
    cmd.yaw_gains.kp = arg_float(query, "yawKp", yaw_gains.kp);
    cmd.yaw_gains.ki = arg_float(query, "yawKi", yaw_gains.ki);
    cmd.yaw_gains.kd = arg_float(query, "yawKd", yaw_gains.kd);

    flight_control_apply_command(&cmd);
    apply = arg_equals(query, "apply", "1");
    stabilize = arg_equals(query, "stabilize", "1");
    flight_control_mark_command();

    if (apply && stabilize)
    {
        if (!flight_control_task_running())
        {
            return send_text(req, "503 Service Unavailable", "Tarefa de controle indisponivel");
        }
        if (flight_control_failsafe_latched())
        {
            return send_text(req, "409 Conflict",
                             "Failsafe ativo; execute PARAR TUDO antes de reabilitar");
        }
        flight_control_engage_stabilized();
    }
    else
    {
        if (flight_control_is_enabled())
        {
            flight_control_disable(FLIGHT_FAILSAFE_MANUAL_OVERRIDE, true);
        }
        flight_control_compute_mix(apply);
    }
    return send_text(req, "200 OK", "OK");
}

/** @brief GET /resetPid : zera os controladores PID (mixagem e cascata). */
static esp_err_t reset_pid_handler(httpd_req_t *req)
{
    flight_control_reset_pid();
    return send_text(req, "200 OK", "PID reiniciado");
}

/**
 * @brief GET /setVerticalHold?enable=0|1 : liga/desliga a malha de velocidade vertical.
 *
 * Funcionalidade adicional (sec. 11). Quando ligada e com a malha estabilizada
 * ativa, o stick de throttle passa a comandar velocidade vertical (climb-rate).
 * Recusa 423 durante o arming.
 */
static esp_err_t set_vertical_hold_handler(httpd_req_t *req)
{
    char query[64];

    if (app_state_is_arming())
    {
        return send_text(req, "423 Locked", "Motores ainda armando");
    }
    (void)read_query(req, query, sizeof(query));
    /* Parametros independentes e opcionais: so age no que estiver presente. */
    if (arg_present(query, "enable"))
    {
        flight_control_set_vertical_hold(arg_equals(query, "enable", "1"));
    }
    if (arg_present(query, "baroGuard"))
    {
        flight_control_set_vertical_baro_guard(arg_equals(query, "baroGuard", "1"));
    }
    return send_text(req, "200 OK", "Controle vertical atualizado");
}

/**
 * @brief GET /setVerticalGains?kp=&ki=&vmax= : sintoniza o PI vertical em runtime.
 *
 * Campos omitidos mantem o valor atual. Os ganhos sao aplicados imediatamente e
 * persistidos na NVS (sobrevivem a reboot), para sintonia iterativa em voo.
 */
static esp_err_t set_vertical_gains_handler(httpd_req_t *req)
{
    char query[64];
    vertical_control_t v;
    float kp;
    float ki;
    float vmax;

    flight_control_get_vertical(&v);
    (void)read_query(req, query, sizeof(query));
    kp = arg_float(query, "kp", v.kp_us_per_ms);
    ki = arg_float(query, "ki", v.ki_us_per_ms);
    vmax = arg_float(query, "vmax", v.max_velocity_ms);
    flight_control_set_vertical_gains(kp, ki, vmax);
    flight_control_get_vertical(&v);
    vertical_params_save(v.kp_us_per_ms, v.ki_us_per_ms, v.max_velocity_ms);
    return send_text(req, "200 OK", "Ganhos verticais atualizados e salvos");
}

/**
 * @brief GET /flightStatus : estado completo da malha (JSON).
 *
 * Inclui setpoints, atitude, taxas, ganhos, correcoes, saidas, motivo de
 * failsafe e prontidao de altitude/GPS. E a fonte da telemetria de voo da
 * interface.
 */
static esp_err_t flight_status_handler(httpd_req_t *req)
{
    char buf[1100];
    size_t off = 0U;
    const size_t cap = sizeof(buf);
    flight_status_t st;
    vertical_control_t vert;
    pid_gains_t rg;
    pid_gains_t pg;
    pid_gains_t yg;
    bmp280_data_t bmp;
    neo6m_data_t gps;
    bool altitude_hold_ready;
    bool gps_control_ready;

    (void)memset(&bmp, 0, sizeof(bmp));
    (void)memset(&gps, 0, sizeof(gps));
    altitude_hold_ready = sensor_hub_get_bmp_snapshot(s_hub, &bmp) && bmp.reference_ready &&
                          ((millis() - bmp.updated_at_ms) < 1000U);
    gps_control_ready = sensor_hub_get_gps_snapshot(s_hub, &gps) && !gps.signal_lost &&
                        ((millis() - gps.updated_at_ms) < 2000U);
    flight_control_get_status(&st);
    flight_control_get_gains(&rg, &pg, &yg);
    flight_control_get_vertical(&vert);

    J_APPEND("{\"controlEnabled\":%s,\"failsafe\":\"%s\",\"failsafeLatched\":%s",
             bool_str(flight_control_is_enabled()),
             flight_failsafe_text(flight_control_failsafe_reason()),
             bool_str(flight_control_failsafe_latched()));
    J_APPEND(",\"commandAgeMs\":%u,\"altitudeHoldReady\":%s,\"gpsControlReady\":%s",
             (unsigned)flight_control_command_age_ms(),
             bool_str(altitude_hold_ready), bool_str(gps_control_ready));
    J_APPEND(",\"setpoint\":{\"throttle\":%.0f,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
             st.setpoint.throttle_us, st.setpoint.roll_deg, st.setpoint.pitch_deg, st.setpoint.yaw_deg);
    J_APPEND(",\"state\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
             st.state.roll_deg, st.state.pitch_deg, st.state.yaw_deg);
    J_APPEND(",\"rate\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
             st.rate.roll_dps, st.rate.pitch_dps, st.rate.yaw_dps);
    J_APPEND(",\"rateSetpoint\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
             st.rate_setpoint.roll_dps, st.rate_setpoint.pitch_dps, st.rate_setpoint.yaw_dps);
    J_APPEND(",\"gains\":{\"roll\":[%.3f,%.3f,%.3f],\"pitch\":[%.3f,%.3f,%.3f],\"yaw\":[%.3f,%.3f,%.3f]}",
             rg.kp, rg.ki, rg.kd, pg.kp, pg.ki, pg.kd, yg.kp, yg.ki, yg.kd);
    J_APPEND(",\"correction\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
             st.correction.roll, st.correction.pitch, st.correction.yaw);
    J_APPEND(",\"motors\":[%d,%d,%d,%d]",
             (int)st.output.m1, (int)st.output.m2, (int)st.output.m3, (int)st.output.m4);
    J_APPEND(",\"vertical\":{\"enabled\":%s,\"engaged\":%s,\"altitudeEstM\":%.2f,"
             "\"velocityEstMs\":%.2f,\"vzSetpointMs\":%.2f,\"accelVertMs2\":%.2f,"
             "\"hoverUs\":%.0f,\"throttleUs\":%.0f,\"saturated\":%s,"
             "\"kp\":%.1f,\"ki\":%.1f,\"vmaxMs\":%.2f,\"baroGuard\":%s}",
             bool_str(flight_control_vertical_hold_enabled()), bool_str(vert.engaged),
             (double)vert.altitude_est_m, (double)vert.velocity_est_ms,
             (double)vert.vz_setpoint_ms, (double)vert.accel_vertical_ms2,
             (double)vert.hover_throttle_us, (double)vert.throttle_output_us,
             bool_str(vert.saturated),
             (double)vert.kp_us_per_ms, (double)vert.ki_us_per_ms,
             (double)vert.max_velocity_ms,
             bool_str(flight_control_vertical_baro_guard_enabled()));
    J_APPEND(",\"arming\":%s}", bool_str(app_state_is_arming()));
    return send_json(req, buf);
}

/**
 * @brief GET /findDeadband?motor=&from=&to=&step=&dwell= : varredura de deadband.
 *
 * Responde antes de executar, pois a varredura e bloqueante (ocupa a tarefa do
 * httpd durante a execucao). Recusa 409 se ja houver varredura em curso e 423
 * durante o arming. Procedimento de bancada, SEM HELICES.
 */
static esp_err_t find_deadband_handler(httpd_req_t *req)
{
    char query[96];
    int32_t motor;
    int32_t from_us;
    int32_t to_us;
    int32_t step_us;
    int32_t dwell_ms;
    esp_err_t result;

    if (app_state_is_arming())
    {
        return send_text(req, "423 Locked", "Motores ainda armando");
    }
    if (esc_deadband_sweep_active())
    {
        return send_text(req, "409 Conflict", "Varredura ja em andamento");
    }
    flight_control_disable(FLIGHT_FAILSAFE_MANUAL_OVERRIDE, true);
    (void)read_query(req, query, sizeof(query));
    if (!arg_present(query, "motor"))
    {
        return send_text(req, "400 Bad Request", "Parametro 'motor' faltando");
    }
    motor = arg_int(query, "motor", 0);
    from_us = arg_int(query, "from", 1000);
    to_us = arg_int(query, "to", 1250);
    step_us = arg_int(query, "step", 2);
    dwell_ms = arg_int(query, "dwell", 400);

    /* Responde antes de executar (a varredura e bloqueante). */
    result = send_text(req, "200 OK",
                       "Varredura iniciada. Observe o motor e leia o log (115200).");
    if ((motor >= 0) && (motor < NUM_MOTORS))
    {
        esc_run_deadband_sweep((uint8_t)motor, from_us, to_us, step_us, dwell_ms);
    }
    return result;
}

/** @brief Resposta padrao para rotas inexistentes (404). */
static esp_err_t not_found_handler(httpd_req_t *req, httpd_err_code_t error)
{
    (void)error;
    return send_text(req, "404 Not Found", "Rota nao encontrada");
}

/** @brief Registra um handler GET para um caminho no servidor. */
static void register_uri(const char *uri, esp_err_t (*handler)(httpd_req_t *))
{
    const httpd_uri_t entry = {
        .uri = uri,
        .method = HTTP_GET,
        .handler = handler,
        .user_ctx = NULL
    };
    (void)httpd_register_uri_handler(s_server, &entry);
}

esp_err_t web_server_start(sensor_hub_t *hub)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    esp_err_t err;

    s_hub = hub;
    config.server_port = 80;
    config.max_uri_handlers = 16;
    config.stack_size = 8192;
    config.lru_purge_enable = true;

    err = httpd_start(&s_server, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar httpd (%s)", esp_err_to_name(err));
        s_server = NULL;
        return err;
    }

    register_uri("/", root_handler);
    register_uri("/setMotor", set_motor_handler);
    register_uri("/stopAll", stop_all_handler);
    register_uri("/status", status_handler);
    register_uri("/health", health_handler);
    register_uri("/sensors", sensors_handler);
    register_uri("/setFlight", set_flight_handler);
    register_uri("/flightStatus", flight_status_handler);
    register_uri("/resetPid", reset_pid_handler);
    register_uri("/setVerticalHold", set_vertical_hold_handler);
    register_uri("/setVerticalGains", set_vertical_gains_handler);
    register_uri("/calibration", calibration_handler);
    register_uri("/setCalibration", set_calibration_handler);
    register_uri("/resetCalibration", reset_calibration_handler);
    register_uri("/findDeadband", find_deadband_handler);
    (void)httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, not_found_handler);

    ESP_LOGI(TAG, "[HTTP] Servidor iniciado na porta 80.");
    return ESP_OK;
}

bool web_server_running(void)
{
    return (s_server != NULL);
}
