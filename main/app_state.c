/**
 * @file app_state.c
 * @brief Implementacao das flags globais de estado (ver app_state.h).
 */
#include "app_state.h"

/* Inicia em true: o boot comeca no periodo de arming dos ESCs. */
static volatile bool s_is_arming = true;
static volatile bool s_telemetry_running = false;

void app_state_set_arming(bool arming)
{
    s_is_arming = arming;
}

bool app_state_is_arming(void)
{
    return s_is_arming;
}

void app_state_set_telemetry_running(bool running)
{
    s_telemetry_running = running;
}

bool app_state_telemetry_running(void)
{
    return s_telemetry_running;
}
