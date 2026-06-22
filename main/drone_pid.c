/**
 * @file drone_pid.c
 * @brief Implementacao dos controladores PID e da mixagem (ver drone_pid.h).
 *
 * As formulas e a ordem das operacoes em ponto flutuante sao definidas com
 * cuidado para garantir saida deterministica.
 */
#include "drone_pid.h"

#include "app_math.h"

/* Saturacao do setpoint de velocidade angular gerado pela malha externa.
 * Limita a agressividade da resposta: 180 graus/s em roll/pitch e um teto
 * seguro para manobra; yaw e mais lento (120 graus/s) por ser menos critico. */
#define MAX_ROLL_PITCH_RATE_DPS  (180.0f)
#define MAX_YAW_RATE_DPS         (120.0f)

/**
 * @brief Normaliza um erro angular para a faixa [-180, 180] graus.
 *
 * Sem isto, o erro de guinada poderia indicar a volta "longa" (ex.: +350 em vez
 * de -10), fazendo o drone girar quase uma volta inteira para corrigir. Garante
 * que a correcao sempre tome o caminho mais curto.
 */
static float wrap_angle_error(float error_deg)
{
    float wrapped = error_deg;
    while (wrapped > 180.0f)
    {
        wrapped -= 360.0f;
    }
    while (wrapped < -180.0f)
    {
        wrapped += 360.0f;
    }
    return wrapped;
}

/* ===== PIDController ===== */
void pid_init(pid_controller_t *pid)
{
    pid->gains.kp = 0.0f;
    pid->gains.ki = 0.0f;
    pid->gains.kd = 0.0f;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output_min = -250.0f;
    pid->output_max = 250.0f;
    pid->integral_limit = 150.0f;
    pid->has_previous_error = false;
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->gains.kp = kp;
    pid->gains.ki = ki;
    pid->gains.kd = kd;
}

void pid_set_output_limits(pid_controller_t *pid, float min_output, float max_output)
{
    pid->output_min = min_output;
    pid->output_max = max_output;
}

void pid_set_integral_limit(pid_controller_t *pid, float limit)
{
    pid->integral_limit = (limit < 0.0f) ? -limit : limit;
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->has_previous_error = false;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement,
                 float dt_seconds)
{
    float output;

    if (dt_seconds <= 0.0f)
    {
        output = 0.0f;
    }
    else
    {
        const float error = setpoint - measurement;
        float derivative = 0.0f;

        pid->integral += error * dt_seconds;
        pid->integral = clamp_f(pid->integral, -pid->integral_limit, pid->integral_limit);

        if (pid->has_previous_error)
        {
            derivative = (error - pid->previous_error) / dt_seconds;
        }
        pid->previous_error = error;
        pid->has_previous_error = true;

        output = (pid->gains.kp * error) + (pid->gains.ki * pid->integral) +
                 (pid->gains.kd * derivative);
        output = clamp_f(output, pid->output_min, pid->output_max);
    }

    return output;
}

/* ===== QuadPIDController ===== */
void quad_pid_init(quad_pid_controller_t *quad)
{
    pid_init(&quad->roll_pid);
    pid_init(&quad->pitch_pid);
    pid_init(&quad->yaw_pid);
    quad_pid_set_roll_gains(quad, 4.0f, 0.02f, 0.9f);
    quad_pid_set_pitch_gains(quad, 4.0f, 0.02f, 0.9f);
    quad_pid_set_yaw_gains(quad, 2.2f, 0.01f, 0.3f);
    quad_pid_set_output_limits(quad, -250.0f, 250.0f);
    quad_pid_set_integral_limit(quad, 120.0f);
}

void quad_pid_set_roll_gains(quad_pid_controller_t *quad, float kp, float ki, float kd)
{
    pid_set_gains(&quad->roll_pid, kp, ki, kd);
}

void quad_pid_set_pitch_gains(quad_pid_controller_t *quad, float kp, float ki, float kd)
{
    pid_set_gains(&quad->pitch_pid, kp, ki, kd);
}

void quad_pid_set_yaw_gains(quad_pid_controller_t *quad, float kp, float ki, float kd)
{
    pid_set_gains(&quad->yaw_pid, kp, ki, kd);
}

void quad_pid_set_output_limits(quad_pid_controller_t *quad, float min_us, float max_us)
{
    pid_set_output_limits(&quad->roll_pid, min_us, max_us);
    pid_set_output_limits(&quad->pitch_pid, min_us, max_us);
    pid_set_output_limits(&quad->yaw_pid, min_us, max_us);
}

void quad_pid_set_integral_limit(quad_pid_controller_t *quad, float limit)
{
    pid_set_integral_limit(&quad->roll_pid, limit);
    pid_set_integral_limit(&quad->pitch_pid, limit);
    pid_set_integral_limit(&quad->yaw_pid, limit);
}

void quad_pid_reset(quad_pid_controller_t *quad)
{
    pid_reset(&quad->roll_pid);
    pid_reset(&quad->pitch_pid);
    pid_reset(&quad->yaw_pid);
}

axis_correction_t quad_pid_compute(quad_pid_controller_t *quad,
                                   const flight_setpoint_t *setpoint,
                                   const flight_state_t *state, float dt_seconds)
{
    axis_correction_t correction;
    correction.roll = pid_update(&quad->roll_pid, setpoint->roll_deg, state->roll_deg, dt_seconds);
    correction.pitch = pid_update(&quad->pitch_pid, setpoint->pitch_deg, state->pitch_deg, dt_seconds);
    correction.yaw = pid_update(&quad->yaw_pid, setpoint->yaw_deg, state->yaw_deg, dt_seconds);
    return correction;
}

quad_motor_output_t quad_pid_mix_x(float throttle_us, const axis_correction_t *correction,
                                   int32_t esc_min, int32_t esc_max)
{
    quad_motor_output_t output;

    /* Sinais da mixagem em X por posicao do motor:
     *   pitch+ acelera a frente (M1,M2) e desacelera a tras (M3,M4);
     *   roll+  acelera a esquerda (M1,M4) e desacelera a direita (M2,M3);
     *   yaw+   acelera o par de rotacao horaria (M2,M4) e desacelera o anti-
     *          horario (M1,M3), gerando torque de guinada.
     * O cast (int32_t) trunca em direcao a zero (parte da definicao da saida). */
    output.m1 = clamp_i32((int32_t)(throttle_us + correction->pitch + correction->roll - correction->yaw),
                          esc_min, esc_max);
    output.m2 = clamp_i32((int32_t)(throttle_us + correction->pitch - correction->roll + correction->yaw),
                          esc_min, esc_max);
    output.m3 = clamp_i32((int32_t)(throttle_us - correction->pitch - correction->roll - correction->yaw),
                          esc_min, esc_max);
    output.m4 = clamp_i32((int32_t)(throttle_us - correction->pitch + correction->roll + correction->yaw),
                          esc_min, esc_max);
    return output;
}

/**
 * @brief Maior fator em [0,1] que mantem (base + fator*delta) dentro de [min,max].
 *
 * Usado para escalar a parcela de yaw a folga disponivel em cada motor: o yaw e
 * o eixo de menor prioridade, entao cede primeiro quando falta faixa. Assume que
 * 'base' (parcela de coletivo+roll+pitch) ja esta dentro de [min,max].
 */
static float yaw_fit_scale(float base, float delta, float min_us, float max_us)
{
    float scale = 1.0f;
    if (delta > 0.0f)
    {
        const float headroom = max_us - base; /* >= 0: base ja cabe */
        if (headroom <= 0.0f)        { scale = 0.0f; }
        else if (delta > headroom)   { scale = headroom / delta; }
        else                         { scale = 1.0f; }
    }
    else if (delta < 0.0f)
    {
        const float headroom = min_us - base; /* <= 0: base ja cabe */
        if (headroom >= 0.0f)        { scale = 0.0f; }
        else if (delta < headroom)   { scale = headroom / delta; } /* dois negativos -> [0,1) */
        else                         { scale = 1.0f; }
    }
    else
    {
        scale = 1.0f;
    }
    return scale;
}

/**
 * @brief Mixagem em X com desaturacao priorizada (roll/pitch > coletivo > yaw).
 *
 * Em condicao normal (sem saturacao) o resultado e identico a ::quad_pid_mix_x.
 * Quando o comando nao cabe na faixa do ESC, a prioridade preserva a autoridade
 * de atitude mais critica para a estabilidade:
 *   1. Se o proprio diferencial de roll/pitch excede a faixa total, escala
 *      roll/pitch juntos (limite fisico; mantem a direcao do torque).
 *   2. Desloca o coletivo comum (air-mode) para encaixar roll/pitch sem perder o
 *      diferencial.
 *   3. Adiciona o yaw escalado a folga restante (yaw e o primeiro a ceder).
 * O clamp final e apenas rede de seguranca; 'saturated' sinaliza qualquer perda
 * de autoridade e 'collective_adjust_us' reporta o deslocamento de coletivo.
 */
quad_mix_result_t quad_pid_mix_x_desaturated(float throttle_us,
                                             const axis_correction_t *correction,
                                             int32_t esc_min, int32_t esc_max)
{
    quad_mix_result_t result;
    const float min_us = (float)esc_min;
    const float max_us = (float)esc_max;
    const float range = max_us - min_us;
    const float yaw = correction->yaw;
    /* Parte de coletivo+roll+pitch (sem yaw). Sinais de roll/pitch como na X. */
    float b1 = throttle_us + correction->pitch + correction->roll;
    float b2 = throttle_us + correction->pitch - correction->roll;
    float b3 = throttle_us - correction->pitch - correction->roll;
    float b4 = throttle_us - correction->pitch + correction->roll;
    float bmax;
    float bmin;
    float shift = 0.0f;
    float yaw_scale;
    float yd;

    result.saturated = false;

    /* 1) Diferencial de roll/pitch maior que a faixa total: escala proporcional. */
    bmax = b1; if (b2 > bmax) { bmax = b2; } if (b3 > bmax) { bmax = b3; } if (b4 > bmax) { bmax = b4; }
    bmin = b1; if (b2 < bmin) { bmin = b2; } if (b3 < bmin) { bmin = b3; } if (b4 < bmin) { bmin = b4; }
    if ((bmax - bmin) > range)
    {
        const float scale = range / (bmax - bmin);
        b1 = throttle_us + ((b1 - throttle_us) * scale);
        b2 = throttle_us + ((b2 - throttle_us) * scale);
        b3 = throttle_us + ((b3 - throttle_us) * scale);
        b4 = throttle_us + ((b4 - throttle_us) * scale);
        result.saturated = true;
    }

    /* 2) Desloca o coletivo para encaixar roll/pitch em [min,max] (air-mode). */
    bmax = b1; if (b2 > bmax) { bmax = b2; } if (b3 > bmax) { bmax = b3; } if (b4 > bmax) { bmax = b4; }
    bmin = b1; if (b2 < bmin) { bmin = b2; } if (b3 < bmin) { bmin = b3; } if (b4 < bmin) { bmin = b4; }
    if (bmax > max_us)
    {
        shift = max_us - bmax;
    }
    if ((bmin + shift) < min_us)
    {
        shift = min_us - bmin;
    }
    if (shift != 0.0f)
    {
        b1 += shift;
        b2 += shift;
        b3 += shift;
        b4 += shift;
        result.saturated = true;
    }

    /* 3) Yaw escalado a folga restante (sinais m1-,m2+,m3-,m4+). */
    yaw_scale = yaw_fit_scale(b1, -yaw, min_us, max_us);
    {
        float s;
        s = yaw_fit_scale(b2, yaw, min_us, max_us);  if (s < yaw_scale) { yaw_scale = s; }
        s = yaw_fit_scale(b3, -yaw, min_us, max_us); if (s < yaw_scale) { yaw_scale = s; }
        s = yaw_fit_scale(b4, yaw, min_us, max_us);  if (s < yaw_scale) { yaw_scale = s; }
    }
    if (yaw_scale < 1.0f)
    {
        result.saturated = true;
    }
    yd = yaw * yaw_scale;
    b1 -= yd;
    b2 += yd;
    b3 -= yd;
    b4 += yd;

    result.output.m1 = clamp_i32((int32_t)b1, esc_min, esc_max);
    result.output.m2 = clamp_i32((int32_t)b2, esc_min, esc_max);
    result.output.m3 = clamp_i32((int32_t)b3, esc_min, esc_max);
    result.output.m4 = clamp_i32((int32_t)b4, esc_min, esc_max);
    result.collective_adjust_us = shift;
    return result;
}

/* ===== AttitudeRateController ===== */
void attitude_rate_init(attitude_rate_controller_t *ctrl)
{
    pid_init(&ctrl->roll_rate_pid);
    pid_init(&ctrl->pitch_rate_pid);
    pid_init(&ctrl->yaw_rate_pid);
    ctrl->rate_setpoint.roll_dps = 0.0f;
    ctrl->rate_setpoint.pitch_dps = 0.0f;
    ctrl->rate_setpoint.yaw_dps = 0.0f;
    ctrl->roll_attitude_kp = 4.0f;
    ctrl->pitch_attitude_kp = 4.0f;
    ctrl->yaw_attitude_kp = 2.0f;
    pid_set_gains(&ctrl->roll_rate_pid, 1.20f, 0.02f, 0.015f);
    pid_set_gains(&ctrl->pitch_rate_pid, 1.20f, 0.02f, 0.015f);
    pid_set_gains(&ctrl->yaw_rate_pid, 1.00f, 0.01f, 0.0f);
    attitude_rate_set_output_limits(ctrl, -220.0f, 220.0f);
    pid_set_integral_limit(&ctrl->roll_rate_pid, 80.0f);
    pid_set_integral_limit(&ctrl->pitch_rate_pid, 80.0f);
    pid_set_integral_limit(&ctrl->yaw_rate_pid, 60.0f);
}

void attitude_rate_reset(attitude_rate_controller_t *ctrl)
{
    pid_reset(&ctrl->roll_rate_pid);
    pid_reset(&ctrl->pitch_rate_pid);
    pid_reset(&ctrl->yaw_rate_pid);
    ctrl->rate_setpoint.roll_dps = 0.0f;
    ctrl->rate_setpoint.pitch_dps = 0.0f;
    ctrl->rate_setpoint.yaw_dps = 0.0f;
}

void attitude_rate_set_attitude_gains(attitude_rate_controller_t *ctrl,
                                      float roll_kp, float pitch_kp, float yaw_kp)
{
    ctrl->roll_attitude_kp = roll_kp;
    ctrl->pitch_attitude_kp = pitch_kp;
    ctrl->yaw_attitude_kp = yaw_kp;
}

void attitude_rate_set_output_limits(attitude_rate_controller_t *ctrl,
                                     float min_us, float max_us)
{
    pid_set_output_limits(&ctrl->roll_rate_pid, min_us, max_us);
    pid_set_output_limits(&ctrl->pitch_rate_pid, min_us, max_us);
    pid_set_output_limits(&ctrl->yaw_rate_pid, min_us, max_us);
}

axis_correction_t attitude_rate_compute(attitude_rate_controller_t *ctrl,
                                        const flight_setpoint_t *setpoint,
                                        const flight_state_t *attitude,
                                        const angular_rate_state_t *rate,
                                        float dt_seconds)
{
    axis_correction_t correction;
    const float roll_error = setpoint->roll_deg - attitude->roll_deg;
    const float pitch_error = setpoint->pitch_deg - attitude->pitch_deg;
    const float yaw_error = wrap_angle_error(setpoint->yaw_deg - attitude->yaw_deg);

    ctrl->rate_setpoint.roll_dps =
        clamp_f(roll_error * ctrl->roll_attitude_kp, -MAX_ROLL_PITCH_RATE_DPS, MAX_ROLL_PITCH_RATE_DPS);
    ctrl->rate_setpoint.pitch_dps =
        clamp_f(pitch_error * ctrl->pitch_attitude_kp, -MAX_ROLL_PITCH_RATE_DPS, MAX_ROLL_PITCH_RATE_DPS);
    ctrl->rate_setpoint.yaw_dps =
        clamp_f(yaw_error * ctrl->yaw_attitude_kp, -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);

    correction.roll = pid_update(&ctrl->roll_rate_pid, ctrl->rate_setpoint.roll_dps, rate->roll_dps, dt_seconds);
    correction.pitch = pid_update(&ctrl->pitch_rate_pid, ctrl->rate_setpoint.pitch_dps, rate->pitch_dps, dt_seconds);
    correction.yaw = pid_update(&ctrl->yaw_rate_pid, ctrl->rate_setpoint.yaw_dps, rate->yaw_dps, dt_seconds);
    return correction;
}

axis_correction_t attitude_rate_compute_yaw_rate(attitude_rate_controller_t *ctrl,
                                                const flight_setpoint_t *setpoint,
                                                const flight_state_t *attitude,
                                                const angular_rate_state_t *rate,
                                                float yaw_rate_setpoint_dps,
                                                float dt_seconds)
{
    axis_correction_t correction;
    const float roll_error = setpoint->roll_deg - attitude->roll_deg;
    const float pitch_error = setpoint->pitch_deg - attitude->pitch_deg;

    ctrl->rate_setpoint.roll_dps =
        clamp_f(roll_error * ctrl->roll_attitude_kp, -MAX_ROLL_PITCH_RATE_DPS, MAX_ROLL_PITCH_RATE_DPS);
    ctrl->rate_setpoint.pitch_dps =
        clamp_f(pitch_error * ctrl->pitch_attitude_kp, -MAX_ROLL_PITCH_RATE_DPS, MAX_ROLL_PITCH_RATE_DPS);
    ctrl->rate_setpoint.yaw_dps =
        clamp_f(yaw_rate_setpoint_dps, -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);

    correction.roll = pid_update(&ctrl->roll_rate_pid, ctrl->rate_setpoint.roll_dps, rate->roll_dps, dt_seconds);
    correction.pitch = pid_update(&ctrl->pitch_rate_pid, ctrl->rate_setpoint.pitch_dps, rate->pitch_dps, dt_seconds);
    correction.yaw = pid_update(&ctrl->yaw_rate_pid, ctrl->rate_setpoint.yaw_dps, rate->yaw_dps, dt_seconds);
    return correction;
}

angular_rate_setpoint_t attitude_rate_setpoint(const attitude_rate_controller_t *ctrl)
{
    return ctrl->rate_setpoint;
}
