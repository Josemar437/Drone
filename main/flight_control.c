/**
 * @file flight_control.c
 * @brief Implementacao da malha de controle e dos failsafes (ver flight_control.h).
 *
 * A sequencia de verificacoes de seguranca, a mixagem e os tempos sao criticos
 * para a estabilidade. O estado e protegido por dois mecanismos: uma secao
 * critica (s_data_mutex) para os dados de voo trocados entre tarefas e um mutex
 * (s_controller_mutex) para o acesso aos controladores, cujo compute nao deve
 * ocorrer concorrentemente com mudancas de ganho.
 */
#include "flight_control.h"

#include <math.h>
#include <string.h>
#include "config.h"
#include "esc_pwm.h"
#include "calibration_store.h"
#include "app_time.h"
#include "app_math.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define FLIGHT_CONTROL_CORE       (1)
#define FLIGHT_CONTROL_STACK      (4096)
#define FLIGHT_CONTROL_PRIORITY   (2)

#define YAW_RATE_COMMAND_MAX_DPS  (90.0f)
#define YAW_RATE_SLEW_DPS2        (180.0f)
#define YAW_RATE_DEADBAND_DPS     (2.0f)
#define TAKEOFF_RAMP_US_PER_S     (180.0f)
#define THROTTLE_SLEW_US_PER_S    (260.0f)
#define HOVER_STICK_BAND_US       (80.0f)
#define LEVEL_SETPOINT_BAND_DEG   (2.0f)
#define HOVER_VELOCITY_BAND_MS    (0.35f)
#define CONTROL_DT_MIN_SECONDS    (0.005f)
#define CONTROL_DT_MAX_SECONDS    (0.100f)

static const char *TAG = "flight";

/* ===== Estado da malha de controle ===== */
static sensor_hub_t *s_hub = NULL;

static quad_pid_controller_t s_flight_controller;
static attitude_rate_controller_t s_attitude_rate_controller;

static flight_setpoint_t s_setpoint = { (float)ESC_MIN_US, 0.0f, 0.0f, 0.0f };
static flight_state_t s_state = { 0.0f, 0.0f, 0.0f };
static angular_rate_state_t s_rate_state = { 0.0f, 0.0f, 0.0f };
static angular_rate_setpoint_t s_last_rate_setpoint = { 0.0f, 0.0f, 0.0f };
static axis_correction_t s_last_correction = { 0.0f, 0.0f, 0.0f };
static quad_motor_output_t s_last_output = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };
static flight_phase_t s_flight_phase = FLIGHT_PHASE_DISARMED;
static float s_yaw_rate_command_dps = 0.0f;
static float s_yaw_rate_active_dps = 0.0f;
/* s_collective_slew_us mantem o estado da rampa/slew do coletivo do operador.
 * s_collective_throttle_us e o coletivo efetivamente aplicado antes da mixagem
 * (telemetria): coincide com o slew, exceto com a malha vertical engatada, onde
 * reflete a saida da malha de velocidade vertical. */
static float s_collective_slew_us = (float)ESC_MIN_US;
static float s_collective_throttle_us = (float)ESC_MIN_US;
static bool s_last_mixer_saturated = false;
static float s_last_mixer_collective_adjust_us = 0.0f;
static float s_control_dt_seconds = (float)FLIGHT_CONTROL_INTERVAL_MS / 1000.0f;
static float s_loop_jitter_ms = 0.0f;
static uint32_t s_last_loop_ms = 0U;
static uint32_t s_imu_age_ms = 0U;
static uint32_t s_last_processed_mpu_seq = 0U;
static uint32_t s_repeated_imu_samples = 0U;

static pid_gains_t s_roll_gains = { ROLL_GAIN_KP_DEFAULT, ROLL_GAIN_KI_DEFAULT, ROLL_GAIN_KD_DEFAULT };
static pid_gains_t s_pitch_gains = { PITCH_GAIN_KP_DEFAULT, PITCH_GAIN_KI_DEFAULT, PITCH_GAIN_KD_DEFAULT };
static pid_gains_t s_yaw_gains = { YAW_GAIN_KP_DEFAULT, YAW_GAIN_KI_DEFAULT, YAW_GAIN_KD_DEFAULT };

static uint32_t s_last_flight_update_ms = 0U;
static volatile uint32_t s_last_flight_command_ms = 0U;
static volatile bool s_control_enabled = false;
static volatile bool s_failsafe_latched = false;
static volatile flight_failsafe_reason_t s_failsafe_reason = FLIGHT_FAILSAFE_NONE;

static portMUX_TYPE s_data_mutex = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_controller_mutex = NULL;
static TaskHandle_t s_task_handle = NULL;

/* Controle de velocidade vertical (sec. 11). Desligado por padrao; protegido
 * pela mesma seccao critica dos dados de voo (acessado pela tarefa e pelo web). */
static vertical_control_t s_vertical;
static volatile bool s_vertical_hold_enabled = false;
/* Trava de seguranca: quando ligada, o controle vertical so atua com a
 * referencia do barometro pronta e recente. Habilitavel/desabilitavel. */
static volatile bool s_vertical_baro_guard_enabled = true;
/* Pedido de ganhos verticais vindo do web. A escrita de s_vertical e exclusiva
 * da tarefa de controle; os handlers HTTP apenas encostam estes campos (sob
 * secao critica) e a tarefa os aplica no proximo ciclo (single-writer). */
static volatile bool s_vertical_gains_pending = false;
static float s_vertical_gains_kp = 0.0f;
static float s_vertical_gains_ki = 0.0f;
static float s_vertical_gains_vmax = 0.0f;

const char *flight_failsafe_text(flight_failsafe_reason_t reason)
{
    const char *text;
    switch (reason)
    {
        case FLIGHT_FAILSAFE_NONE:                text = "NONE"; break;
        case FLIGHT_FAILSAFE_COMMAND_TIMEOUT:     text = "COMMAND_TIMEOUT"; break;
        case FLIGHT_FAILSAFE_MPU_INVALID:         text = "MPU_INVALID"; break;
        case FLIGHT_FAILSAFE_MPU_NOT_CALIBRATED:  text = "MPU_NOT_CALIBRATED"; break;
        case FLIGHT_FAILSAFE_EXCESSIVE_TILT:      text = "EXCESSIVE_TILT"; break;
        case FLIGHT_FAILSAFE_MANUAL_OVERRIDE:     text = "MANUAL_OVERRIDE"; break;
        case FLIGHT_FAILSAFE_EMERGENCY_STOP:      text = "EMERGENCY_STOP"; break;
        case FLIGHT_FAILSAFE_CONTROLLER_TIMEOUT:  text = "CONTROLLER_TIMEOUT"; break;
        default:                                  text = "UNKNOWN"; break;
    }
    return text;
}

const char *flight_phase_text(flight_phase_t phase)
{
    const char *text;
    switch (phase)
    {
        case FLIGHT_PHASE_DISARMED:     text = "DISARMED"; break;
        case FLIGHT_PHASE_ARMING:       text = "ARMING"; break;
        case FLIGHT_PHASE_IDLE:         text = "IDLE"; break;
        case FLIGHT_PHASE_TAKEOFF_RAMP: text = "TAKEOFF_RAMP"; break;
        case FLIGHT_PHASE_HOVER:        text = "HOVER"; break;
        case FLIGHT_PHASE_FLIGHT:       text = "FLIGHT"; break;
        case FLIGHT_PHASE_FAILSAFE:     text = "FAILSAFE"; break;
        default:                        text = "UNKNOWN"; break;
    }
    return text;
}

static float slew_toward(float current, float target, float rate_per_s, float dt_seconds)
{
    const float step = rate_per_s * dt_seconds;
    float next = current;
    if (target > (current + step))
    {
        next = current + step;
    }
    else if (target < (current - step))
    {
        next = current - step;
    }
    else
    {
        next = target;
    }
    return next;
}

static bool setpoint_is_hover_like(const flight_setpoint_t *setpoint)
{
    const bool vertical_stable = s_vertical.initialized &&
                                 (fabsf(s_vertical.velocity_est_ms) <= HOVER_VELOCITY_BAND_MS);
    return (fabsf(setpoint->roll_deg) <= LEVEL_SETPOINT_BAND_DEG) &&
           (fabsf(setpoint->pitch_deg) <= LEVEL_SETPOINT_BAND_DEG) &&
           (fabsf(setpoint->throttle_us - VERT_HOVER_THROTTLE_US_DEFAULT) <= HOVER_STICK_BAND_US) &&
           (fabsf(s_yaw_rate_active_dps) <= YAW_RATE_DEADBAND_DPS) &&
           vertical_stable;
}

static float control_dt_from_mpu(const mpu9259_data_t *mpu)
{
    float dt_seconds = mpu->dt_seconds;
    if ((s_last_processed_mpu_seq != 0U) && (mpu->updated_at_ms > s_last_flight_update_ms))
    {
        dt_seconds = (float)(mpu->updated_at_ms - s_last_flight_update_ms) / 1000.0f;
    }
    if ((dt_seconds < CONTROL_DT_MIN_SECONDS) || (dt_seconds > CONTROL_DT_MAX_SECONDS))
    {
        dt_seconds = (float)FLIGHT_CONTROL_INTERVAL_MS / 1000.0f;
    }
    return dt_seconds;
}

static bool update_yaw_rate_command(float target_dps, float measured_yaw_deg,
                                    float dt_seconds, flight_setpoint_t *setpoint,
                                    float *yaw_rate_setpoint_dps)
{
    const float previous_active = s_yaw_rate_active_dps;
    float commanded = clamp_f(target_dps, -YAW_RATE_COMMAND_MAX_DPS, YAW_RATE_COMMAND_MAX_DPS);
    if (fabsf(commanded) < YAW_RATE_DEADBAND_DPS)
    {
        commanded = 0.0f;
    }
    s_yaw_rate_active_dps = slew_toward(s_yaw_rate_active_dps, commanded,
                                        YAW_RATE_SLEW_DPS2, dt_seconds);
    if (fabsf(s_yaw_rate_active_dps) <= YAW_RATE_DEADBAND_DPS)
    {
        s_yaw_rate_active_dps = 0.0f;
        if (fabsf(previous_active) > YAW_RATE_DEADBAND_DPS)
        {
            setpoint->yaw_deg = measured_yaw_deg;
        }
    }
    else
    {
        setpoint->yaw_deg = measured_yaw_deg;
    }
    *yaw_rate_setpoint_dps = s_yaw_rate_active_dps;
    return (s_yaw_rate_active_dps != 0.0f);
}

static float update_collective_throttle(const flight_setpoint_t *setpoint, float dt_seconds)
{
    float target = clamp_f(setpoint->throttle_us, (float)ESC_MIN_US, (float)ESC_MAX_US);
    float rate = THROTTLE_SLEW_US_PER_S;

    if (target <= (float)CONTROL_MIN_THROTTLE_US)
    {
        s_collective_slew_us = (float)ESC_MIN_US;
        s_flight_phase = FLIGHT_PHASE_IDLE;
        return s_collective_slew_us;
    }

    if ((s_flight_phase == FLIGHT_PHASE_IDLE) ||
        (s_flight_phase == FLIGHT_PHASE_DISARMED))
    {
        s_collective_slew_us = (float)CONTROL_MIN_THROTTLE_US;
        s_flight_phase = FLIGHT_PHASE_TAKEOFF_RAMP;
    }

    if (s_flight_phase == FLIGHT_PHASE_TAKEOFF_RAMP)
    {
        rate = TAKEOFF_RAMP_US_PER_S;
    }

    s_collective_slew_us = slew_toward(s_collective_slew_us, target, rate, dt_seconds);
    s_collective_slew_us = clamp_f(s_collective_slew_us, (float)ESC_MIN_US, (float)ESC_MAX_US);

    if ((s_flight_phase == FLIGHT_PHASE_TAKEOFF_RAMP) &&
        (fabsf(s_collective_slew_us - target) < 1.0f))
    {
        s_flight_phase = setpoint_is_hover_like(setpoint) ? FLIGHT_PHASE_HOVER : FLIGHT_PHASE_FLIGHT;
    }
    else if ((s_flight_phase == FLIGHT_PHASE_HOVER) && !setpoint_is_hover_like(setpoint))
    {
        s_flight_phase = FLIGHT_PHASE_FLIGHT;
    }
    else if ((s_flight_phase == FLIGHT_PHASE_FLIGHT) && setpoint_is_hover_like(setpoint))
    {
        s_flight_phase = FLIGHT_PHASE_HOVER;
    }
    else
    {
        /* Mantem o estado atual. */
    }

    return s_collective_slew_us;
}

/** @brief Propaga os ganhos correntes para o controlador de mixagem simples. */
static void refresh_flight_controller_gains(void)
{
    quad_pid_set_roll_gains(&s_flight_controller, s_roll_gains.kp, s_roll_gains.ki, s_roll_gains.kd);
    quad_pid_set_pitch_gains(&s_flight_controller, s_pitch_gains.kp, s_pitch_gains.ki, s_pitch_gains.kd);
    quad_pid_set_yaw_gains(&s_flight_controller, s_yaw_gains.kp, s_yaw_gains.ki, s_yaw_gains.kd);
}

/**
 * @brief Reinicia a cascata sob o mutex dos controladores.
 *
 * Protegido por mutex porque pode ser chamado pelo web e pela tarefa de
 * controle; o timeout evita bloqueio indefinido caso o mutex esteja ocupado.
 */
static void reset_attitude_rate_controller(void)
{
    if ((s_controller_mutex != NULL) &&
        (xSemaphoreTake(s_controller_mutex, pdMS_TO_TICKS(50)) == pdTRUE))
    {
        attitude_rate_reset(&s_attitude_rate_controller);
        (void)xSemaphoreGive(s_controller_mutex);
    }
}

/** @brief Atualiza os Kp da malha externa da cascata sob o mutex dos controladores. */
static void update_attitude_outer_gains(void)
{
    if ((s_controller_mutex != NULL) &&
        (xSemaphoreTake(s_controller_mutex, pdMS_TO_TICKS(50)) == pdTRUE))
    {
        attitude_rate_set_attitude_gains(&s_attitude_rate_controller,
                                         s_roll_gains.kp, s_pitch_gains.kp, s_yaw_gains.kp);
        (void)xSemaphoreGive(s_controller_mutex);
    }
}

/**
 * @brief Executa um passo da cascata sob o mutex dos controladores.
 *
 * Usa timeout curto (compativel com o ciclo de 20 ms): se nao conseguir o mutex,
 * retorna false e o chamador trata como falha (failsafe), em vez de atrasar o laco.
 *
 * @param setpoint             Angulos desejados (roll/pitch/yaw).
 * @param state                Atitude atual estimada.
 * @param rate                 Velocidade angular medida pelo giroscopio.
 * @param yaw_rate_mode        true: yaw em taxa direta; false: heading-hold.
 * @param yaw_rate_setpoint_dps Setpoint de taxa de yaw (dps), usado se yaw_rate_mode.
 * @param dt_seconds           Intervalo desde o ultimo passo (s).
 * @param correction           Saida: correcao por eixo.
 * @param rate_setpoint        Saida: setpoint de taxa calculado.
 * @return true se calculou; false se o mutex nao pode ser obtido.
 */
static bool compute_attitude_rate_correction(
    const flight_setpoint_t *setpoint, const flight_state_t *state,
    const angular_rate_state_t *rate, bool yaw_rate_mode,
    float yaw_rate_setpoint_dps, float dt_seconds,
    axis_correction_t *correction, angular_rate_setpoint_t *rate_setpoint)
{
    if ((s_controller_mutex == NULL) ||
        (xSemaphoreTake(s_controller_mutex, pdMS_TO_TICKS(5)) != pdTRUE))
    {
        return false;
    }
    if (yaw_rate_mode)
    {
        *correction = attitude_rate_compute_yaw_rate(&s_attitude_rate_controller, setpoint, state,
                                                     rate, yaw_rate_setpoint_dps, dt_seconds);
    }
    else
    {
        *correction = attitude_rate_compute(&s_attitude_rate_controller, setpoint, state, rate, dt_seconds);
    }
    *rate_setpoint = attitude_rate_setpoint(&s_attitude_rate_controller);
    (void)xSemaphoreGive(s_controller_mutex);
    return true;
}

void flight_control_disable(flight_failsafe_reason_t reason, bool stop_motors)
{
    const bool was_enabled = s_control_enabled;
    s_control_enabled = false;
    s_failsafe_reason = reason;
    s_flight_phase = (reason == FLIGHT_FAILSAFE_MANUAL_OVERRIDE) ?
        FLIGHT_PHASE_DISARMED : FLIGHT_PHASE_FAILSAFE;
    s_yaw_rate_command_dps = 0.0f;
    s_yaw_rate_active_dps = 0.0f;
    s_collective_slew_us = (float)ESC_MIN_US;
    s_collective_throttle_us = (float)ESC_MIN_US;
    if (was_enabled || stop_motors)
    {
        ESP_LOGI(TAG, "[CONTROL] Desabilitado. Motivo: %s | parar motores: %s",
                 flight_failsafe_text(reason), stop_motors ? "SIM" : "NAO");
    }
    if ((reason == FLIGHT_FAILSAFE_COMMAND_TIMEOUT) ||
        (reason == FLIGHT_FAILSAFE_MPU_INVALID) ||
        (reason == FLIGHT_FAILSAFE_MPU_NOT_CALIBRATED) ||
        (reason == FLIGHT_FAILSAFE_EXCESSIVE_TILT) ||
        ((reason == FLIGHT_FAILSAFE_MANUAL_OVERRIDE) && was_enabled) ||
        (reason == FLIGHT_FAILSAFE_EMERGENCY_STOP) ||
        (reason == FLIGHT_FAILSAFE_CONTROLLER_TIMEOUT))
    {
        s_failsafe_latched = true;
    }
    reset_attitude_rate_controller();
    quad_pid_reset(&s_flight_controller);
    if (stop_motors)
    {
        uint8_t motor;
        for (motor = 0U; motor < (uint8_t)NUM_MOTORS; motor++)
        {
            esc_set_motor_speed_quiet(motor, ESC_MIN_US);
        }
        taskENTER_CRITICAL(&s_data_mutex);
        s_last_correction.roll = 0.0f;
        s_last_correction.pitch = 0.0f;
        s_last_correction.yaw = 0.0f;
        s_last_rate_setpoint.roll_dps = 0.0f;
        s_last_rate_setpoint.pitch_dps = 0.0f;
        s_last_rate_setpoint.yaw_dps = 0.0f;
        s_last_output.m1 = ESC_MIN_US;
        s_last_output.m2 = ESC_MIN_US;
        s_last_output.m3 = ESC_MIN_US;
        s_last_output.m4 = ESC_MIN_US;
        s_collective_slew_us = (float)ESC_MIN_US;
        s_collective_throttle_us = (float)ESC_MIN_US;
        taskEXIT_CRITICAL(&s_data_mutex);
    }
}

/**
 * @brief Mapeia um snapshot da IMU para atitude/taxas no frame do controle.
 *
 * Aplica a troca roll<->pitch (CONTROL_SWAP_ROLL_PITCH) que alinha os eixos da IMU
 * aos eixos do frame conforme a montagem fisica. Funcao pura (so escreve nas
 * saidas): usada tanto pelo caminho de controle quanto pelo autoteste de eixos,
 * garantindo que ambos usem exatamente a mesma convencao.
 */
static void map_mpu_to_state(const mpu9259_data_t *mpu, flight_state_t *state,
                             angular_rate_state_t *rate)
{
    if (CONTROL_SWAP_ROLL_PITCH)
    {
        state->roll_deg = mpu->pitch_deg;
        state->pitch_deg = mpu->roll_deg;
        rate->roll_dps = mpu->gyro_y;
        rate->pitch_dps = mpu->gyro_x;
    }
    else
    {
        state->roll_deg = mpu->roll_deg;
        state->pitch_deg = mpu->pitch_deg;
        rate->roll_dps = mpu->gyro_x;
        rate->pitch_dps = mpu->gyro_y;
    }
    state->yaw_deg = mpu->yaw_deg;
    rate->yaw_dps = mpu->gyro_z;
}

/**
 * @brief Atualiza o estado de voo (atitude e taxas) a partir de um snapshot da IMU.
 *
 * Rejeita amostras invalidas ou velhas demais (mais que MPU_MAX_AGE_MS) e publica
 * o estado mapeado (ver ::map_mpu_to_state) sob secao critica.
 *
 * @return true se o estado foi atualizado; false se a amostra nao e utilizavel.
 */
static bool update_flight_state_from_mpu(const mpu9259_data_t *mpu)
{
    const uint32_t age_ms = (mpu->updated_at_ms != 0U) ?
        (millis() - mpu->updated_at_ms) : (MPU_MAX_AGE_MS + 1U);
    flight_state_t mapped_state;
    angular_rate_state_t mapped_rate;

    if (!mpu->valid || (age_ms > MPU_MAX_AGE_MS))
    {
        return false;
    }

    map_mpu_to_state(mpu, &mapped_state, &mapped_rate);

    taskENTER_CRITICAL(&s_data_mutex);
    s_state = mapped_state;
    s_rate_state = mapped_rate;
    taskEXIT_CRITICAL(&s_data_mutex);
    return true;
}

/**
 * @brief Leva os motores ao minimo e zera as saidas quando o throttle e baixo.
 *
 * Com throttle abaixo do minimo de controle, nao ha empuxo para estabilizar;
 * manter os motores no minimo (e reiniciar a cascata) evita correcoes inuteis e
 * windup enquanto o operador ainda nao deu aceleracao.
 */
static void zero_outputs_at_idle(void)
{
    uint8_t motor;
    reset_attitude_rate_controller();
    for (motor = 0U; motor < (uint8_t)NUM_MOTORS; motor++)
    {
        esc_set_motor_speed_quiet(motor, ESC_MIN_US);
    }
    taskENTER_CRITICAL(&s_data_mutex);
    s_last_correction.roll = 0.0f;
    s_last_correction.pitch = 0.0f;
    s_last_correction.yaw = 0.0f;
    s_last_rate_setpoint.roll_dps = 0.0f;
    s_last_rate_setpoint.pitch_dps = 0.0f;
    s_last_rate_setpoint.yaw_dps = 0.0f;
    s_last_output.m1 = ESC_MIN_US;
    s_last_output.m2 = ESC_MIN_US;
    s_last_output.m3 = ESC_MIN_US;
    s_last_output.m4 = ESC_MIN_US;
    s_flight_phase = FLIGHT_PHASE_IDLE;
    s_yaw_rate_active_dps = 0.0f;
    s_collective_slew_us = (float)ESC_MIN_US;
    s_collective_throttle_us = (float)ESC_MIN_US;
    s_last_mixer_saturated = false;
    s_last_mixer_collective_adjust_us = 0.0f;
    taskEXIT_CRITICAL(&s_data_mutex);
}

/**
 * @brief Tarefa de tempo real do controle de voo (periodo fixo de 20 ms).
 *
 * A cada ciclo, com o controle habilitado: le a IMU, verifica em ordem as
 * condicoes de seguranca (timeout de comando, throttle minimo, IMU valida e
 * calibrada, inclinacao segura) e, estando tudo certo, calcula a cascata, mistura
 * e escreve nos ESCs. O periodo fixo (vTaskDelayUntil) mantem o dt do PID estavel.
 */
static void flight_control_task(void *parameter)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    bool prev_vertical_engaged = false;
    (void)parameter;

    for (;;)
    {
        bool baro_ready_ok = false;
        bool vertical_effective;
        const uint32_t loop_now = millis();
        /* Periodo real entre iteracoes da tarefa = dt entre chamadas do estimador
         * vertical (chamado uma vez por ciclo). Independe da taxa da IMU. */
        const uint32_t loop_period_ms = (s_last_loop_ms != 0U) ?
            (loop_now - s_last_loop_ms) : (uint32_t)FLIGHT_CONTROL_INTERVAL_MS;
        float loop_dt_seconds = (float)loop_period_ms / 1000.0f;

        if (s_last_loop_ms != 0U)
        {
            s_loop_jitter_ms = (float)loop_period_ms - (float)FLIGHT_CONTROL_INTERVAL_MS;
        }
        s_last_loop_ms = loop_now;
        if ((loop_dt_seconds < CONTROL_DT_MIN_SECONDS) || (loop_dt_seconds > CONTROL_DT_MAX_SECONDS))
        {
            loop_dt_seconds = (float)FLIGHT_CONTROL_INTERVAL_MS / 1000.0f;
        }

        /* Aplica ganhos verticais pedidos pelo web (single-writer: s_vertical so
         * e escrito aqui, na tarefa). */
        if (s_vertical_gains_pending)
        {
            float pend_kp;
            float pend_ki;
            float pend_vmax;
            taskENTER_CRITICAL(&s_data_mutex);
            pend_kp = s_vertical_gains_kp;
            pend_ki = s_vertical_gains_ki;
            pend_vmax = s_vertical_gains_vmax;
            s_vertical_gains_pending = false;
            taskEXIT_CRITICAL(&s_data_mutex);
            vertical_control_set_gains(&s_vertical, pend_kp, pend_ki, pend_vmax);
        }

        /* Estimador vertical roda sempre (telemetria/validacao), sem atuar nos
         * motores enquanto desengatado. So a tarefa escreve s_vertical. */
        {
            mpu9259_data_t est_mpu;
            bmp280_data_t est_bmp;
            (void)memset(&est_mpu, 0, sizeof(est_mpu));
            (void)memset(&est_bmp, 0, sizeof(est_bmp));
            if (sensor_hub_get_bmp_snapshot(s_hub, &est_bmp))
            {
                baro_ready_ok = est_bmp.reference_ready && (est_bmp.updated_at_ms != 0U) &&
                                ((millis() - est_bmp.updated_at_ms) < VERT_BARO_MAX_AGE_MS);
            }
            if (sensor_hub_get_mpu_snapshot(s_hub, &est_mpu) && est_mpu.valid)
            {
                /* dt = periodo real do laco (cadencia do estimador), nao o dt por
                 * amostra da IMU: a IMU agora roda mais rapido que esta tarefa. */
                vertical_estimator_update(&s_vertical, est_mpu.accel_x, est_mpu.accel_y,
                                          est_mpu.accel_z, est_mpu.roll_deg, est_mpu.pitch_deg,
                                          est_bmp.altitude_m, baro_ready_ok, loop_dt_seconds);
            }
        }

        /* Engate efetivo = pedido do operador E (trava desligada OU baro pronto).
         * Com a trava ligada, se o baro falhar/atrasar a malha desengata e o
         * throttle volta ao operador (sem salto). Re-engata bumpless ao voltar.
         * Todo o estado s_vertical e escrito so aqui (na tarefa). */
        vertical_effective = s_vertical_hold_enabled &&
                             ((!s_vertical_baro_guard_enabled) || baro_ready_ok);
        if (vertical_effective != prev_vertical_engaged)
        {
            if (vertical_effective)
            {
                float current_throttle;
                taskENTER_CRITICAL(&s_data_mutex);
                current_throttle = s_setpoint.throttle_us;
                taskEXIT_CRITICAL(&s_data_mutex);
                vertical_control_engage(&s_vertical, current_throttle);
            }
            else
            {
                vertical_control_disengage(&s_vertical);
            }
            prev_vertical_engaged = vertical_effective;
        }

        if (s_control_enabled)
        {
            const uint32_t now = millis();
            mpu9259_data_t mpu;
            bool mpu_valid;
            bool repeated_mpu_sample = false;
            flight_setpoint_t setpoint;
            flight_state_t state;
            angular_rate_state_t rate;
            float yaw_rate_command;

            (void)memset(&mpu, 0, sizeof(mpu));
            mpu_valid = sensor_hub_get_mpu_snapshot(s_hub, &mpu);
            if (mpu_valid)
            {
                s_imu_age_ms = (mpu.updated_at_ms != 0U) ? (now - mpu.updated_at_ms) : UINT32_MAX;
                repeated_mpu_sample = (mpu.valid && (mpu.sample_seq == s_last_processed_mpu_seq));
                if (!repeated_mpu_sample)
                {
                    mpu_valid = update_flight_state_from_mpu(&mpu);
                }
            }

            taskENTER_CRITICAL(&s_data_mutex);
            setpoint = s_setpoint;
            state = s_state;
            rate = s_rate_state;
            yaw_rate_command = s_yaw_rate_command_dps;
            taskEXIT_CRITICAL(&s_data_mutex);

            if ((now - s_last_flight_command_ms) > FLIGHT_COMMAND_TIMEOUT_MS)
            {
                flight_control_disable(FLIGHT_FAILSAFE_COMMAND_TIMEOUT, true);
            }
            else if (setpoint.throttle_us <= (float)CONTROL_MIN_THROTTLE_US)
            {
                zero_outputs_at_idle();
            }
            else if (repeated_mpu_sample && (s_imu_age_ms <= MPU_MAX_AGE_MS))
            {
                s_repeated_imu_samples++;
            }
            else if (repeated_mpu_sample)
            {
                flight_control_disable(FLIGHT_FAILSAFE_MPU_INVALID, true);
            }
            else if (!mpu_valid)
            {
                flight_control_disable(FLIGHT_FAILSAFE_MPU_INVALID, true);
            }
            else if (!mpu.calibration_complete)
            {
                flight_control_disable(FLIGHT_FAILSAFE_MPU_NOT_CALIBRATED, true);
            }
            else if ((fabsf(state.roll_deg) > MAX_SAFE_TILT_DEG) ||
                     (fabsf(state.pitch_deg) > MAX_SAFE_TILT_DEG))
            {
                flight_control_disable(FLIGHT_FAILSAFE_EXCESSIVE_TILT, true);
            }
            else
            {
                const float dt_seconds = control_dt_from_mpu(&mpu);
                axis_correction_t correction = { 0.0f, 0.0f, 0.0f };
                angular_rate_setpoint_t rate_setpoint = { 0.0f, 0.0f, 0.0f };
                float yaw_rate_setpoint = 0.0f;
                const bool yaw_rate_mode =
                    update_yaw_rate_command(yaw_rate_command, state.yaw_deg,
                                            dt_seconds, &setpoint, &yaw_rate_setpoint);
                if (!compute_attitude_rate_correction(&setpoint, &state, &rate,
                                                      yaw_rate_mode, yaw_rate_setpoint, dt_seconds,
                                                      &correction, &rate_setpoint))
                {
                    /* Mutex da cascata indisponivel no prazo: o passo de controle
                     * nao foi calculado. Reportado separadamente de IMU invalida
                     * para nao mascarar a causa real (contencao do controlador). */
                    flight_control_disable(FLIGHT_FAILSAFE_CONTROLLER_TIMEOUT, true);
                    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(FLIGHT_CONTROL_INTERVAL_MS));
                    continue;
                }
                {
                    /* Empuxo base: direto do operador (caminho congelado) ou da
                     * malha de velocidade vertical, quando habilitada. */
                    float throttle_cmd = update_collective_throttle(&setpoint, dt_seconds);
                    quad_mix_result_t mix;
                    const bool previous_mixer_saturated = s_last_mixer_saturated;
                    if (s_vertical.engaged)
                    {
                        const float vz_setpoint =
                            vertical_throttle_to_setpoint(setpoint.throttle_us,
                                                          s_vertical.max_velocity_ms);
                        throttle_cmd = vertical_velocity_hold(&s_vertical, vz_setpoint,
                                                              state.roll_deg, state.pitch_deg,
                                                              dt_seconds,
                                                              previous_mixer_saturated);
                    }
                    mix = quad_pid_mix_x_desaturated(throttle_cmd, &correction,
                                                     ESC_MIN_US, ESC_MAX_US);
                    taskENTER_CRITICAL(&s_data_mutex);
                    s_last_correction = correction;
                    s_last_rate_setpoint = rate_setpoint;
                    s_last_output = mix.output;
                    s_setpoint.yaw_deg = setpoint.yaw_deg;
                    /* Coletivo aplicado antes da mixagem: reflete a saida da malha
                     * vertical quando engatada, ou o slew do operador caso contrario. */
                    s_collective_throttle_us = throttle_cmd;
                    s_last_mixer_saturated = mix.saturated;
                    s_last_mixer_collective_adjust_us = mix.collective_adjust_us;
                    s_control_dt_seconds = dt_seconds;
                    s_last_flight_update_ms = mpu.updated_at_ms;
                    s_last_processed_mpu_seq = mpu.sample_seq;
                    taskEXIT_CRITICAL(&s_data_mutex);
                    if (s_control_enabled)
                    {
                        esc_set_motor_speed_quiet(0U, mix.output.m1);
                        esc_set_motor_speed_quiet(1U, mix.output.m2);
                        esc_set_motor_speed_quiet(2U, mix.output.m3);
                        esc_set_motor_speed_quiet(3U, mix.output.m4);
                    }
                }
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(FLIGHT_CONTROL_INTERVAL_MS));
    }
}

void flight_control_init(sensor_hub_t *hub)
{
    s_hub = hub;
    quad_pid_init(&s_flight_controller);
    attitude_rate_init(&s_attitude_rate_controller);
    vertical_control_init(&s_vertical);
    s_flight_phase = FLIGHT_PHASE_DISARMED;
    /* Aplica os ganhos verticais salvos na NVS (mantem defaults se ausentes). */
    {
        float kp = s_vertical.kp_us_per_ms;
        float ki = s_vertical.ki_us_per_ms;
        float vmax = s_vertical.max_velocity_ms;
        vertical_params_load(&kp, &ki, &vmax);
        vertical_control_set_gains(&s_vertical, kp, ki, vmax);
    }
    refresh_flight_controller_gains();

    s_controller_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "[INIT] Mutex da malha de controle: %s",
             (s_controller_mutex != NULL) ? "OK" : "FALHA");
    update_attitude_outer_gains();
}

bool flight_control_start(void)
{
    const BaseType_t result = xTaskCreatePinnedToCore(
        flight_control_task, "FlightControl", FLIGHT_CONTROL_STACK, NULL,
        FLIGHT_CONTROL_PRIORITY, &s_task_handle, FLIGHT_CONTROL_CORE);
    ESP_LOGI(TAG, "[TASK] FlightControl: %s", (result == pdPASS) ? "OK" : "FALHA");
    return (result == pdPASS);
}

bool flight_control_task_running(void)
{
    return (s_task_handle != NULL);
}

bool flight_control_is_enabled(void)
{
    return s_control_enabled;
}

bool flight_control_failsafe_latched(void)
{
    return s_failsafe_latched;
}

flight_failsafe_reason_t flight_control_failsafe_reason(void)
{
    return s_failsafe_reason;
}

uint32_t flight_control_command_age_ms(void)
{
    return (s_last_flight_command_ms != 0U) ? (millis() - s_last_flight_command_ms) : 0U;
}

void flight_control_get_gains(pid_gains_t *roll, pid_gains_t *pitch, pid_gains_t *yaw)
{
    if ((roll != NULL) && (pitch != NULL) && (yaw != NULL))
    {
        *roll = s_roll_gains;
        *pitch = s_pitch_gains;
        *yaw = s_yaw_gains;
    }
}

void flight_control_get_status(flight_status_t *status)
{
    if (status == NULL)
    {
        return;
    }
    taskENTER_CRITICAL(&s_data_mutex);
    status->setpoint = s_setpoint;
    status->state = s_state;
    status->rate = s_rate_state;
    status->rate_setpoint = s_last_rate_setpoint;
    status->correction = s_last_correction;
    status->output = s_last_output;
    status->phase = s_flight_phase;
    status->yaw_rate_command_dps = s_yaw_rate_command_dps;
    status->yaw_rate_active_dps = s_yaw_rate_active_dps;
    status->collective_throttle_us = s_collective_throttle_us;
    status->mixer_saturated = s_last_mixer_saturated;
    status->mixer_collective_adjust_us = s_last_mixer_collective_adjust_us;
    status->control_dt_ms = s_control_dt_seconds * 1000.0f;
    status->loop_jitter_ms = s_loop_jitter_ms;
    status->imu_age_ms = s_imu_age_ms;
    status->imu_sample_seq = s_last_processed_mpu_seq;
    status->repeated_imu_samples = s_repeated_imu_samples;
    taskEXIT_CRITICAL(&s_data_mutex);
}

bool flight_control_axis_self_test(axis_self_test_t *out)
{
    mpu9259_data_t mpu;
    flight_state_t state = { 0.0f, 0.0f, 0.0f };
    angular_rate_state_t rate = { 0.0f, 0.0f, 0.0f };
    flight_setpoint_t sp;
    attitude_rate_controller_t preview;
    axis_correction_t correction = { 0.0f, 0.0f, 0.0f };
    quad_mix_result_t mix;
    const float hover = (float)VERT_HOVER_THROTTLE_US_DEFAULT;
    const float dt = (float)FLIGHT_CONTROL_INTERVAL_MS / 1000.0f;

    if (out == NULL)
    {
        return false;
    }

    (void)memset(&mpu, 0, sizeof(mpu));
    out->hover_throttle_us = hover;
    out->mpu_ok = false;
    out->calibrated = false;
    out->state = state;
    out->rate = rate;
    out->correction = correction;
    out->output.m1 = ESC_MIN_US;
    out->output.m2 = ESC_MIN_US;
    out->output.m3 = ESC_MIN_US;
    out->output.m4 = ESC_MIN_US;
    out->motor_delta_us[0] = 0.0f;
    out->motor_delta_us[1] = 0.0f;
    out->motor_delta_us[2] = 0.0f;
    out->motor_delta_us[3] = 0.0f;

    if (!sensor_hub_get_mpu_snapshot(s_hub, &mpu) || !mpu.valid)
    {
        return false;
    }

    /* Mesma convencao de eixos do caminho de controle (helper compartilhado),
     * porem em variaveis locais: nao escreve s_state nem perturba a malha ativa. */
    map_mpu_to_state(&mpu, &state, &rate);

    /* Setpoint nivelado: a correcao resultante opoe-se a inclinacao medida.
     * Yaw alvo = yaw atual (erro de rumo zero); o eixo de yaw responde a taxa. */
    sp.throttle_us = hover;
    sp.roll_deg = 0.0f;
    sp.pitch_deg = 0.0f;
    sp.yaw_deg = state.yaw_deg;

    /* Controlador temporario (estado zerado), com os mesmos ganhos correntes: a
     * cadeia de sinais e identica a do voo, mas sem efeito colateral. */
    attitude_rate_init(&preview);
    attitude_rate_set_attitude_gains(&preview, s_roll_gains.kp, s_pitch_gains.kp, s_yaw_gains.kp);
    correction = attitude_rate_compute(&preview, &sp, &state, &rate, dt);

    mix = quad_pid_mix_x_desaturated(hover, &correction, ESC_MIN_US, ESC_MAX_US);

    out->mpu_ok = true;
    out->calibrated = mpu.calibration_complete;
    out->state = state;
    out->rate = rate;
    out->correction = correction;
    out->output = mix.output;
    out->motor_delta_us[0] = (float)mix.output.m1 - hover;
    out->motor_delta_us[1] = (float)mix.output.m2 - hover;
    out->motor_delta_us[2] = (float)mix.output.m3 - hover;
    out->motor_delta_us[3] = (float)mix.output.m4 - hover;
    return true;
}

void flight_control_apply_command(const flight_command_t *cmd)
{
    if (cmd == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(&s_data_mutex);
    s_setpoint.throttle_us = cmd->throttle_us;
    s_setpoint.roll_deg = cmd->roll_setpoint_deg;
    s_setpoint.pitch_deg = cmd->pitch_setpoint_deg;
    s_yaw_rate_command_dps = clamp_f(cmd->yaw_setpoint_deg,
                                     -YAW_RATE_COMMAND_MAX_DPS,
                                     YAW_RATE_COMMAND_MAX_DPS);
    if (!s_control_enabled)
    {
        s_setpoint.yaw_deg = cmd->yaw_setpoint_deg;
        s_yaw_rate_command_dps = 0.0f;
        s_yaw_rate_active_dps = 0.0f;
        s_state.roll_deg = cmd->manual_roll_deg;
        s_state.pitch_deg = cmd->manual_pitch_deg;
        s_state.yaw_deg = cmd->manual_yaw_deg;
    }
    taskEXIT_CRITICAL(&s_data_mutex);

    s_roll_gains = cmd->roll_gains;
    s_pitch_gains = cmd->pitch_gains;
    s_yaw_gains = cmd->yaw_gains;
    refresh_flight_controller_gains();
    update_attitude_outer_gains();
}

void flight_control_mark_command(void)
{
    s_last_flight_command_ms = millis();
}

void flight_control_engage_stabilized(void)
{
    if (!s_control_enabled)
    {
        mpu9259_data_t mpu;
        (void)memset(&mpu, 0, sizeof(mpu));
        if (sensor_hub_get_mpu_snapshot(s_hub, &mpu) && mpu.valid)
        {
            taskENTER_CRITICAL(&s_data_mutex);
            s_setpoint.yaw_deg = mpu.yaw_deg;
            s_yaw_rate_command_dps = 0.0f;
            s_yaw_rate_active_dps = 0.0f;
            s_collective_slew_us = (float)ESC_MIN_US;
            s_collective_throttle_us = (float)ESC_MIN_US;
            s_flight_phase = FLIGHT_PHASE_IDLE;
            taskEXIT_CRITICAL(&s_data_mutex);
        }
        reset_attitude_rate_controller();
    }
    s_failsafe_reason = FLIGHT_FAILSAFE_NONE;
    if (s_flight_phase == FLIGHT_PHASE_DISARMED)
    {
        s_flight_phase = FLIGHT_PHASE_IDLE;
    }
    s_control_enabled = true;
}

void flight_control_compute_mix(bool apply_to_motors)
{
    const uint32_t now = millis();
    float dt = (s_last_flight_update_ms == 0U) ? 0.02f :
               ((float)(now - s_last_flight_update_ms) / 1000.0f);
    flight_setpoint_t setpoint;
    flight_state_t state;
    axis_correction_t correction;
    quad_motor_output_t output;

    if ((dt <= 0.0f) || (dt > 0.5f))
    {
        dt = 0.02f;
    }
    s_last_flight_update_ms = now;

    taskENTER_CRITICAL(&s_data_mutex);
    setpoint = s_setpoint;
    state = s_state;
    taskEXIT_CRITICAL(&s_data_mutex);

    correction = quad_pid_compute(&s_flight_controller, &setpoint, &state, dt);
    output = quad_pid_mix_x(setpoint.throttle_us, &correction, ESC_MIN_US, ESC_MAX_US);

    taskENTER_CRITICAL(&s_data_mutex);
    s_last_correction = correction;
    s_last_output = output;
    taskEXIT_CRITICAL(&s_data_mutex);

    if (apply_to_motors)
    {
        esc_set_motor_speed(0U, output.m1);
        esc_set_motor_speed(1U, output.m2);
        esc_set_motor_speed(2U, output.m3);
        esc_set_motor_speed(3U, output.m4);
    }
}

void flight_control_reset_pid(void)
{
    quad_pid_reset(&s_flight_controller);
    reset_attitude_rate_controller();
    s_last_flight_update_ms = 0U;
    s_yaw_rate_active_dps = 0.0f;
}

void flight_control_clear_failsafe_latch(void)
{
    s_failsafe_latched = false;
    if (!s_control_enabled)
    {
        s_flight_phase = FLIGHT_PHASE_DISARMED;
    }
}

void flight_control_set_vertical_hold(bool enable)
{
    /* Apenas sinaliza; o engate/desengate efetivo (que altera s_vertical) ocorre
     * na tarefa de controle, mantendo a escrita de s_vertical num unico contexto. */
    s_vertical_hold_enabled = enable;
}

bool flight_control_vertical_hold_enabled(void)
{
    return s_vertical_hold_enabled;
}

void flight_control_get_vertical(vertical_control_t *out)
{
    if (out != NULL)
    {
        *out = s_vertical;
    }
}

void flight_control_set_vertical_gains(float kp_us_per_ms, float ki_us_per_ms,
                                       float max_velocity_ms)
{
    /* Apenas encosta o pedido; a tarefa de controle aplica em s_vertical no
     * proximo ciclo, mantendo a escrita de s_vertical num unico contexto. */
    taskENTER_CRITICAL(&s_data_mutex);
    s_vertical_gains_kp = kp_us_per_ms;
    s_vertical_gains_ki = ki_us_per_ms;
    s_vertical_gains_vmax = max_velocity_ms;
    s_vertical_gains_pending = true;
    taskEXIT_CRITICAL(&s_data_mutex);
}

void flight_control_set_vertical_baro_guard(bool enable)
{
    s_vertical_baro_guard_enabled = enable;
}

bool flight_control_vertical_baro_guard_enabled(void)
{
    return s_vertical_baro_guard_enabled;
}
