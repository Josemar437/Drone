/**
 * @file vertical_control.c
 * @brief Implementacao do estimador vertical e da malha de velocidade vertical.
 *
 * Ver vertical_control.h. As constantes de sintonia/limites estao em config.h.
 */
#include "vertical_control.h"

#include <math.h>
#include "config.h"
#include "app_math.h"

void vertical_control_init(vertical_control_t *vc)
{
    vc->altitude_est_m = 0.0f;
    vc->velocity_est_ms = 0.0f;
    vc->accel_vertical_ms2 = 0.0f;
    vc->initialized = false;
    vc->kp_us_per_ms = VERT_VEL_KP_US_PER_MS;
    vc->ki_us_per_ms = VERT_VEL_KI_US_PER_MS;
    vc->max_velocity_ms = VERT_MAX_VELOCITY_MS;
    vc->integral_us = 0.0f;
    vc->hover_throttle_us = VERT_HOVER_THROTTLE_US_DEFAULT;
    vc->vz_setpoint_ms = 0.0f;
    vc->throttle_output_us = (float)ESC_MIN_US;
    vc->engaged = false;
    vc->saturated = false;
}

void vertical_estimator_update(vertical_control_t *vc, float ax_g, float ay_g, float az_g,
                               float roll_deg, float pitch_deg, float baro_alt_m,
                               bool baro_valid, float dt_seconds)
{
    float roll_rad;
    float pitch_rad;
    float az_world_g;
    float correction_error;

    if (dt_seconds <= 0.0f)
    {
        return;
    }

    roll_rad = roll_deg * DEG_TO_RAD_F;
    pitch_rad = pitch_deg * DEG_TO_RAD_F;

    /* Componente do vetor aceleracao (forca especifica) ao longo do eixo vertical
     * do mundo (Z para cima), obtida projetando o accel do corpo via roll/pitch.
     * Convencao: com o drone nivelado, az_g ~ +1 -> az_world_g ~ +1. */
    az_world_g = (-sinf(pitch_rad) * ax_g) +
                 (cosf(pitch_rad) * sinf(roll_rad) * ay_g) +
                 (cosf(pitch_rad) * cosf(roll_rad) * az_g);

    /* Remove a gravidade (1 g) e converte para m/s^2: aceleracao linear vertical. */
    vc->accel_vertical_ms2 = (az_world_g - 1.0f) * GRAVITY_MS2;

    if (!vc->initialized)
    {
        vc->altitude_est_m = baro_valid ? baro_alt_m : 0.0f;
        vc->velocity_est_ms = 0.0f;
        vc->initialized = true;
    }

    /* Filtro complementar: integra a aceleracao e corrige com o barometro.
     *   z' = v + K1*(baro - z)
     *   v' = a + K2*(baro - z)
     * Sem baro valido, faz dead-reckoning (apenas integra o accel). */
    correction_error = baro_valid ? (baro_alt_m - vc->altitude_est_m) : 0.0f;
    vc->altitude_est_m += (vc->velocity_est_ms + (VERT_EST_K1 * correction_error)) * dt_seconds;
    vc->velocity_est_ms += (vc->accel_vertical_ms2 + (VERT_EST_K2 * correction_error)) * dt_seconds;
}

void vertical_control_engage(vertical_control_t *vc, float current_throttle_us)
{
    /* Bumpless: integral zerado e hover = empuxo corrente -> saida inicial igual
     * ao empuxo aplicado (sem degrau no motor ao engatar). */
    vc->integral_us = 0.0f;
    vc->hover_throttle_us = clamp_f(current_throttle_us, (float)ESC_MIN_US, (float)ESC_MAX_US);
    vc->engaged = true;
}

void vertical_control_disengage(vertical_control_t *vc)
{
    vc->engaged = false;
}

void vertical_control_set_gains(vertical_control_t *vc, float kp_us_per_ms,
                                float ki_us_per_ms, float max_velocity_ms)
{
    if (kp_us_per_ms > 0.0f)
    {
        vc->kp_us_per_ms = kp_us_per_ms;
    }
    if (ki_us_per_ms >= 0.0f)
    {
        vc->ki_us_per_ms = ki_us_per_ms;
    }
    if (max_velocity_ms > 0.0f)
    {
        vc->max_velocity_ms = max_velocity_ms;
    }
}

float vertical_velocity_hold(vertical_control_t *vc, float vz_setpoint_ms,
                             float roll_deg, float pitch_deg, float dt_seconds,
                             bool mixer_saturated)
{
    float error;
    float p_term;
    float adjust;
    float throttle;
    float tilt_factor;
    float throttle_clamped;
    bool at_output_limit;

    vc->vz_setpoint_ms = clamp_f(vz_setpoint_ms, -vc->max_velocity_ms, vc->max_velocity_ms);
    error = vc->vz_setpoint_ms - vc->velocity_est_ms;

    p_term = vc->kp_us_per_ms * error;
    adjust = clamp_f(p_term + vc->integral_us, -VERT_VEL_OUT_LIMIT_US, VERT_VEL_OUT_LIMIT_US);
    throttle = vc->hover_throttle_us + adjust;

    /* Compensacao de inclinacao: 1/(cosR*cosP), aplicada so a parcela acima do
     * minimo e limitada (cresce demais perto de 90 graus). */
    tilt_factor = cosf(roll_deg * DEG_TO_RAD_F) * cosf(pitch_deg * DEG_TO_RAD_F);
    if (tilt_factor < VERT_TILT_COMP_MIN_COS)
    {
        tilt_factor = VERT_TILT_COMP_MIN_COS;
    }
    throttle = (float)ESC_MIN_US + ((throttle - (float)ESC_MIN_US) / tilt_factor);

    throttle_clamped = clamp_f(throttle, (float)ESC_MIN_US, (float)ESC_MAX_US);
    vc->saturated = (throttle_clamped < throttle) || (throttle_clamped > throttle) || mixer_saturated;

    /* Anti-windup: so integra se a saida nao esta saturada e o termo nao esta
     * pressionando ainda mais o limite de ajuste. */
    at_output_limit = ((error > 0.0f) && (adjust >= VERT_VEL_OUT_LIMIT_US)) ||
                      ((error < 0.0f) && (adjust <= -VERT_VEL_OUT_LIMIT_US));
    if (!vc->saturated && !at_output_limit)
    {
        vc->integral_us += vc->ki_us_per_ms * error * dt_seconds;
        vc->integral_us = clamp_f(vc->integral_us, -VERT_VEL_OUT_LIMIT_US, VERT_VEL_OUT_LIMIT_US);
    }

    vc->throttle_output_us = throttle_clamped;
    return throttle_clamped;
}

float vertical_throttle_to_setpoint(float throttle_us, float max_velocity_ms)
{
    float vz = 0.0f;
    if (throttle_us > VERT_THROTTLE_DEADBAND_HIGH_US)
    {
        vz = ((throttle_us - VERT_THROTTLE_DEADBAND_HIGH_US) /
              ((float)ESC_MAX_US - VERT_THROTTLE_DEADBAND_HIGH_US)) * max_velocity_ms;
    }
    else if (throttle_us < VERT_THROTTLE_DEADBAND_LOW_US)
    {
        vz = ((throttle_us - VERT_THROTTLE_DEADBAND_LOW_US) /
              (VERT_THROTTLE_DEADBAND_LOW_US - (float)ESC_MIN_US)) * max_velocity_ms;
    }
    else
    {
        vz = 0.0f;
    }
    return clamp_f(vz, -max_velocity_ms, max_velocity_ms);
}
