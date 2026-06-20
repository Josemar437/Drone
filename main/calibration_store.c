/**
 * @file calibration_store.c
 * @brief Implementacao da persistencia da calibracao na NVS (ver calibration_store.h).
 *
 * Desvio MISRA (Rule 21.6): snprintf e usado apenas para montar nomes de chave
 * curtos ("start0".."trim3") com tamanho fixo; e a forma delimitada e segura de
 * gerar a chave por indice de motor.
 */
#include "calibration_store.h"

#include <stdio.h>
#include "config.h"
#include "esc_pwm.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define CALIBRATION_NAMESPACE "motor-cal"
#define CALIBRATION_KEY_LEN   (12)
#define VERTICAL_NAMESPACE    "vert-cal"

static const char *TAG = "cal-store";

esp_err_t calibration_store_init(void)
{
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        ESP_LOGW(TAG, "NVS precisa ser apagada (%s). Reinicializando.", esp_err_to_name(err));
        err = nvs_flash_erase();
        if (err == ESP_OK)
        {
            err = nvs_flash_init();
        }
    }
    return err;
}

void calibration_store_load(void)
{
    nvs_handle_t handle;
    const esp_err_t err = nvs_open(CALIBRATION_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Sem calibracao salva (%s). Usando defaults.", esp_err_to_name(err));
        return;
    }

    for (uint8_t motor = 0U; motor < (uint8_t)NUM_MOTORS; motor++)
    {
        char key[CALIBRATION_KEY_LEN];
        int32_t start_us = 0;
        int32_t max_us = 0;
        int32_t trim_us = 0;

        /* Parte do valor default atual e sobrescreve com o que houver na NVS. */
        esc_get_calibration(motor, &start_us, &max_us, &trim_us);

        (void)snprintf(key, sizeof(key), "start%u", (unsigned)motor);
        (void)nvs_get_i32(handle, key, &start_us);
        (void)snprintf(key, sizeof(key), "max%u", (unsigned)motor);
        (void)nvs_get_i32(handle, key, &max_us);
        (void)snprintf(key, sizeof(key), "trim%u", (unsigned)motor);
        (void)nvs_get_i32(handle, key, &trim_us);

        esc_set_calibration(motor, start_us, max_us, trim_us);
    }

    nvs_close(handle);
}

void calibration_store_save(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CALIBRATION_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao abrir NVS para gravacao (%s).", esp_err_to_name(err));
        return;
    }

    for (uint8_t motor = 0U; motor < (uint8_t)NUM_MOTORS; motor++)
    {
        char key[CALIBRATION_KEY_LEN];
        int32_t start_us = 0;
        int32_t max_us = 0;
        int32_t trim_us = 0;

        esc_sanitize_calibration(motor);
        esc_get_calibration(motor, &start_us, &max_us, &trim_us);

        (void)snprintf(key, sizeof(key), "start%u", (unsigned)motor);
        (void)nvs_set_i32(handle, key, start_us);
        (void)snprintf(key, sizeof(key), "max%u", (unsigned)motor);
        (void)nvs_set_i32(handle, key, max_us);
        (void)snprintf(key, sizeof(key), "trim%u", (unsigned)motor);
        (void)nvs_set_i32(handle, key, trim_us);
    }

    err = nvs_commit(handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao gravar calibracao (%s).", esp_err_to_name(err));
    }
    nvs_close(handle);
}

/* Os 3 ganhos verticais sao gravados juntos como um blob ("vparams"). */
typedef struct
{
    float kp_us_per_ms;
    float ki_us_per_ms;
    float max_velocity_ms;
} vertical_params_blob_t;

void vertical_params_load(float *kp_us_per_ms, float *ki_us_per_ms, float *max_velocity_ms)
{
    nvs_handle_t handle;
    vertical_params_blob_t blob;
    size_t length = sizeof(blob);
    esp_err_t err = nvs_open(VERTICAL_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        return; /* Sem valores salvos: mantem os defaults do chamador. */
    }
    if (nvs_get_blob(handle, "vparams", &blob, &length) == ESP_OK)
    {
        if (kp_us_per_ms != NULL)
        {
            *kp_us_per_ms = blob.kp_us_per_ms;
        }
        if (ki_us_per_ms != NULL)
        {
            *ki_us_per_ms = blob.ki_us_per_ms;
        }
        if (max_velocity_ms != NULL)
        {
            *max_velocity_ms = blob.max_velocity_ms;
        }
    }
    nvs_close(handle);
}

void vertical_params_save(float kp_us_per_ms, float ki_us_per_ms, float max_velocity_ms)
{
    nvs_handle_t handle;
    vertical_params_blob_t blob;
    esp_err_t err = nvs_open(VERTICAL_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao abrir NVS vertical (%s).", esp_err_to_name(err));
        return;
    }
    blob.kp_us_per_ms = kp_us_per_ms;
    blob.ki_us_per_ms = ki_us_per_ms;
    blob.max_velocity_ms = max_velocity_ms;
    err = nvs_set_blob(handle, "vparams", &blob, sizeof(blob));
    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao gravar ganhos verticais (%s).", esp_err_to_name(err));
    }
    nvs_close(handle);
}
