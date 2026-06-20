/**
 * @file esc_pwm.c
 * @brief Implementacao do driver de PWM dos ESCs (ver esc_pwm.h).
 *
 * O sinal de servo/ESC e um pulso de 1000..2000 us repetido a 250 Hz. Gera-se
 * com o periferico LEDC: um timer de 16 bits define o periodo (4000 us) e o
 * "duty" de cada canal define a largura do pulso. Calcular o duty a partir da
 * largura desejada (esc_us_to_duty) preserva, em microssegundos, a largura de
 * pulso enviada ao ESC.
 */
#include "esc_pwm.h"

#include <stddef.h>
#include "config.h"
#include "app_math.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ESC_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define ESC_LEDC_TIMER       LEDC_TIMER_0
#define ESC_LEDC_DUTY_RES    LEDC_TIMER_16_BIT
#define ESC_LEDC_DUTY_BITS   (16U)
/* Periodo do sinal em microssegundos (4000 us @ 250 Hz). */
#define ESC_PERIOD_US        (1000000 / ESC_FREQUENCY_HZ)

static const char *TAG = "esc_pwm";

static const int ESC_PINS[NUM_MOTORS] = {
    ESC_GPIO_M1, ESC_GPIO_M2, ESC_GPIO_M3, ESC_GPIO_M4
};

static const ledc_channel_t ESC_CHANNELS[NUM_MOTORS] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3
};

static const int32_t DEFAULT_START_US[NUM_MOTORS] = DEFAULT_MOTOR_START_US_INIT;
static const int32_t DEFAULT_MAX_US[NUM_MOTORS] = DEFAULT_MOTOR_MAX_US_INIT;
static const int32_t DEFAULT_TRIM_US[NUM_MOTORS] = DEFAULT_MOTOR_TRIM_US_INIT;

/* Estado da calibracao e das saidas atuais de cada motor. */
static int32_t s_motor_start_us[NUM_MOTORS] = DEFAULT_MOTOR_START_US_INIT;
static int32_t s_motor_max_us[NUM_MOTORS] = DEFAULT_MOTOR_MAX_US_INIT;
static int32_t s_motor_trim_us[NUM_MOTORS] = DEFAULT_MOTOR_TRIM_US_INIT;
static int32_t s_operating_idle_us = DEFAULT_OPERATING_IDLE_US;
static int32_t s_motor_speed[NUM_MOTORS] = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };
static int32_t s_motor_output_us[NUM_MOTORS] = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };

/**
 * @brief Converte uma largura de pulso (us) no valor de duty do LEDC.
 *
 * duty = pulso / periodo * 2^bits. Com periodo de 4000 us e 16 bits, a
 * resolucao e ~0,06 us por contagem, bem abaixo do que o ESC distingue. O termo
 * (periodo/2) faz arredondamento ao inteiro mais proximo em vez de truncar.
 *
 * @param pulse_us Largura desejada (saturada a ESC_MIN..ESC_MAX).
 * @return Valor de duty a programar no canal LEDC.
 */
static uint32_t esc_us_to_duty(int32_t pulse_us)
{
    const uint32_t max_duty = (uint32_t)1U << ESC_LEDC_DUTY_BITS;
    const int32_t clamped = clamp_i32(pulse_us, ESC_MIN_US, ESC_MAX_US);
    const uint32_t numerator = (uint32_t)clamped * max_duty;
    return (numerator + (uint32_t)(ESC_PERIOD_US / 2)) / (uint32_t)ESC_PERIOD_US;
}

/* Escreve uma largura de pulso bruta (us) diretamente no canal do motor. */
static void esc_write_us(uint8_t motor, int32_t pulse_us)
{
    const uint32_t duty = esc_us_to_duty(pulse_us);
    esp_err_t err = ledc_set_duty(ESC_LEDC_MODE, ESC_CHANNELS[motor], duty);
    if (err == ESP_OK)
    {
        err = ledc_update_duty(ESC_LEDC_MODE, ESC_CHANNELS[motor]);
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Motor %u: falha ao escrever duty (%s)",
                 (unsigned)(motor + 1U), esp_err_to_name(err));
    }
}

esp_err_t esc_pwm_init(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = ESC_LEDC_MODE,
        .duty_resolution = ESC_LEDC_DUTY_RES,
        .timer_num = ESC_LEDC_TIMER,
        .freq_hz = (uint32_t)ESC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao configurar timer LEDC (%s)", esp_err_to_name(err));
        return err;
    }

    for (uint8_t motor = 0U; motor < (uint8_t)NUM_MOTORS; motor++)
    {
        ledc_channel_config_t channel_config = {
            .gpio_num = ESC_PINS[motor],
            .speed_mode = ESC_LEDC_MODE,
            .channel = ESC_CHANNELS[motor],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = ESC_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0
        };

        err = ledc_channel_config(&channel_config);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Motor %u: falha ao configurar canal LEDC no GPIO %d (%s)",
                     (unsigned)(motor + 1U), ESC_PINS[motor], esp_err_to_name(err));
            return err;
        }

        s_motor_speed[motor] = ESC_MIN_US;
        s_motor_output_us[motor] = ESC_MIN_US;
        esc_write_us(motor, ESC_MIN_US);
        ESP_LOGI(TAG, "Motor %u anexado no GPIO %d, canal %d",
                 (unsigned)(motor + 1U), ESC_PINS[motor], (int)ESC_CHANNELS[motor]);
    }

    return ESP_OK;
}

void esc_pwm_set_all_min(void)
{
    for (uint8_t motor = 0U; motor < (uint8_t)NUM_MOTORS; motor++)
    {
        s_motor_speed[motor] = ESC_MIN_US;
        s_motor_output_us[motor] = ESC_MIN_US;
        esc_write_us(motor, ESC_MIN_US);
    }
}

int32_t esc_calibrate_motor_output(uint8_t motor, int32_t command_us)
{
    int32_t result;

    if (motor >= (uint8_t)NUM_MOTORS)
    {
        result = ESC_MIN_US;
    }
    else
    {
        const int32_t logical_us = clamp_i32(command_us, ESC_MIN_US, ESC_MAX_US);
        if (logical_us <= ESC_MIN_US)
        {
            result = ESC_MIN_US;
        }
        else
        {
            int32_t start_us = clamp_i32(s_motor_start_us[motor] + s_motor_trim_us[motor],
                                         ESC_MIN_US, ESC_MAX_US);
            int32_t max_us;
            /* Piso de operacao: eleva o startUs efetivo se idle > 0. */
            if (s_operating_idle_us > 0)
            {
                const int32_t idle = clamp_i32(s_operating_idle_us, ESC_MIN_US, ESC_MAX_US);
                if (idle > start_us)
                {
                    start_us = idle;
                }
            }
            max_us = clamp_i32(s_motor_max_us[motor] + s_motor_trim_us[motor],
                               start_us, ESC_MAX_US);
            result = map_i32(logical_us, ESC_MIN_US + 1, ESC_MAX_US, start_us, max_us);
        }
    }

    return result;
}

void esc_set_motor_speed(uint8_t motor, int32_t speed)
{
    if (motor < (uint8_t)NUM_MOTORS)
    {
        int32_t pct;
        s_motor_speed[motor] = clamp_i32(speed, ESC_MIN_US, ESC_MAX_US);
        s_motor_output_us[motor] = esc_calibrate_motor_output(motor, s_motor_speed[motor]);
        esc_write_us(motor, s_motor_output_us[motor]);

        pct = ((s_motor_speed[motor] - ESC_MIN_US) * 100) / (ESC_MAX_US - ESC_MIN_US);
        ESP_LOGI(TAG, "[Motor %u] %d us logico -> %d us ESC (%d%%)",
                 (unsigned)(motor + 1U), (int)s_motor_speed[motor],
                 (int)s_motor_output_us[motor], (int)pct);
    }
}

void esc_set_motor_speed_quiet(uint8_t motor, int32_t speed)
{
    if (motor < (uint8_t)NUM_MOTORS)
    {
        s_motor_speed[motor] = clamp_i32(speed, ESC_MIN_US, ESC_MAX_US);
        s_motor_output_us[motor] = esc_calibrate_motor_output(motor, s_motor_speed[motor]);
        esc_write_us(motor, s_motor_output_us[motor]);
    }
}

void esc_stop_all_motors(void)
{
    for (uint8_t motor = 0U; motor < (uint8_t)NUM_MOTORS; motor++)
    {
        esc_set_motor_speed(motor, ESC_MIN_US);
    }
    ESP_LOGI(TAG, "Todos os motores parados!");
}

void esc_reapply_current_outputs(void)
{
    for (uint8_t motor = 0U; motor < (uint8_t)NUM_MOTORS; motor++)
    {
        s_motor_output_us[motor] = esc_calibrate_motor_output(motor, s_motor_speed[motor]);
        esc_write_us(motor, s_motor_output_us[motor]);
    }
}

int32_t esc_get_motor_speed(uint8_t motor)
{
    int32_t value = ESC_MIN_US;
    if (motor < (uint8_t)NUM_MOTORS)
    {
        value = s_motor_speed[motor];
    }
    return value;
}

int32_t esc_get_motor_output_us(uint8_t motor)
{
    int32_t value = ESC_MIN_US;
    if (motor < (uint8_t)NUM_MOTORS)
    {
        value = s_motor_output_us[motor];
    }
    return value;
}

void esc_sanitize_calibration(uint8_t motor)
{
    if (motor < (uint8_t)NUM_MOTORS)
    {
        s_motor_trim_us[motor] = clamp_i32(s_motor_trim_us[motor],
                                           MOTOR_TRIM_MIN_US, MOTOR_TRIM_MAX_US);
        s_motor_start_us[motor] = clamp_i32(s_motor_start_us[motor],
                                            ESC_MIN_US, ESC_MAX_US - 1);
        s_motor_max_us[motor] = clamp_i32(s_motor_max_us[motor],
                                          s_motor_start_us[motor] + 1, ESC_MAX_US);
    }
}

void esc_reset_calibration_to_defaults(void)
{
    for (uint8_t motor = 0U; motor < (uint8_t)NUM_MOTORS; motor++)
    {
        s_motor_start_us[motor] = DEFAULT_START_US[motor];
        s_motor_max_us[motor] = DEFAULT_MAX_US[motor];
        s_motor_trim_us[motor] = DEFAULT_TRIM_US[motor];
    }
}

void esc_get_calibration(uint8_t motor, int32_t *start_us, int32_t *max_us, int32_t *trim_us)
{
    if ((motor < (uint8_t)NUM_MOTORS) && (start_us != NULL) &&
        (max_us != NULL) && (trim_us != NULL))
    {
        *start_us = s_motor_start_us[motor];
        *max_us = s_motor_max_us[motor];
        *trim_us = s_motor_trim_us[motor];
    }
}

void esc_set_calibration(uint8_t motor, int32_t start_us, int32_t max_us, int32_t trim_us)
{
    if (motor < (uint8_t)NUM_MOTORS)
    {
        s_motor_start_us[motor] = start_us;
        s_motor_max_us[motor] = max_us;
        s_motor_trim_us[motor] = trim_us;
        esc_sanitize_calibration(motor);
    }
}

void esc_set_operating_idle_us(int32_t idle_us)
{
    s_operating_idle_us = idle_us;
}

int32_t esc_get_operating_idle_us(void)
{
    return s_operating_idle_us;
}

static volatile bool s_deadband_sweep_active = false;

bool esc_deadband_sweep_active(void)
{
    return s_deadband_sweep_active;
}

void esc_run_deadband_sweep(uint8_t motor, int32_t from_us, int32_t to_us,
                            int32_t step_us, int32_t dwell_ms)
{
    uint8_t other;
    int32_t us;

    if (motor >= (uint8_t)NUM_MOTORS)
    {
        return;
    }
    s_deadband_sweep_active = true;

    /* Garante os outros motores parados. */
    for (other = 0U; other < (uint8_t)NUM_MOTORS; other++)
    {
        if (other != motor)
        {
            esc_write_us(other, ESC_MIN_US);
        }
    }

    from_us = clamp_i32(from_us, ESC_MIN_US, ESC_MAX_US);
    to_us = clamp_i32(to_us, from_us, ESC_MAX_US);
    step_us = clamp_i32(step_us, 1, 50);
    dwell_ms = clamp_i32(dwell_ms, 50, 2000);

    ESP_LOGI(TAG, "=== Varredura de deadband - Motor %u (SEM HELICES) ===",
             (unsigned)(motor + 1U));

    for (us = from_us; us <= to_us; us += step_us)
    {
        const int32_t pct = ((us - ESC_MIN_US) * 100) / (ESC_MAX_US - ESC_MIN_US);
        esc_write_us(motor, us);
        ESP_LOGI(TAG, "  pulso %d us (%d%%)", (int)us, (int)pct);
        vTaskDelay(pdMS_TO_TICKS((uint32_t)dwell_ms));
    }

    esc_write_us(motor, ESC_MIN_US);
    s_motor_speed[motor] = ESC_MIN_US;
    s_motor_output_us[motor] = ESC_MIN_US;
    ESP_LOGI(TAG, "=== Fim da varredura. Motor parado. ===");
    s_deadband_sweep_active = false;
}
