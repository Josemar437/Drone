/**
 * @file app_main.c
 * @brief Ponto de entrada do firmware: inicializacao e criacao das tarefas.
 *
 * A ordem de inicializacao e deliberada:
 * calibracao (NVS) -> ESCs -> arming -> sensores -> Wi-Fi AP -> servidor web ->
 * tarefas (telemetria + controle). Os ESCs sao armados antes de subir a rede e o
 * controle, garantindo que os motores fiquem no minimo durante toda a partida.
 */
#include "config.h"
#include "app_state.h"
#include "esc_pwm.h"
#include "calibration_store.h"
#include "sensor_hub.h"
#include "flight_control.h"
#include "wifi_ap.h"
#include "web_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define TELEMETRY_CORE      (1)
#define TELEMETRY_STACK     (4096)
#define TELEMETRY_PRIORITY  (1)

static const char *TAG = "app_main";

/* Instancia unica do agregador de sensores (estado grande -> fora da pilha). */
static sensor_hub_t s_sensor_hub;
static TaskHandle_t s_telemetry_task = NULL;

/**
 * @brief Tarefa de telemetria: le os sensores fora do caminho de tempo real.
 *
 * Mantida separada da tarefa de controle para que a leitura de sensores (que
 * pode bloquear no I2C/UART) nao interfira no laco de controle de 20 ms.
 */
static void telemetry_task(void *parameter)
{
    (void)parameter;
    for (;;)
    {
        sensor_hub_update(&s_sensor_hub);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief Inicializacao do firmware (chamada pelo runtime do ESP-IDF).
 *
 * Executa a sequencia de boot e cria as tarefas. Apos isso, o laco principal
 * fica ocioso: o sistema e tocado pelo httpd e pelas tarefas de telemetria e
 * controle.
 */
void app_main(void)
{
    esp_err_t err;
    BaseType_t task_result;

    ESP_LOGI(TAG, "===== ESP32 MOTOR CONTROLLER (ESP-IDF) =====");

    app_state_set_arming(true);

    err = calibration_store_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar NVS: %s", esp_err_to_name(err));
    }
    calibration_store_load();

    err = esc_pwm_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao inicializar ESCs: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Armando motores - mantendo sinal minimo por %u ms...",
             (unsigned)ESC_ARM_HOLD_MS);
    esc_pwm_set_all_min();
    vTaskDelay(pdMS_TO_TICKS(ESC_ARM_HOLD_MS));
    app_state_set_arming(false);
    ESP_LOGI(TAG, "ESCs armados com sinal minimo!");

    sensor_hub_init(&s_sensor_hub);
    sensor_hub_begin(&s_sensor_hub, I2C_SDA_GPIO, I2C_SCL_GPIO,
                     UART_NUM_2, GPS_RX_GPIO, GPS_TX_GPIO);

    err = wifi_ap_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar Wi-Fi AP: %s", esp_err_to_name(err));
    }

    flight_control_init(&s_sensor_hub);

    err = web_server_start(&s_sensor_hub);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha ao iniciar servidor web: %s", esp_err_to_name(err));
    }

    task_result = xTaskCreatePinnedToCore(
        telemetry_task, "Telemetry", TELEMETRY_STACK, NULL,
        TELEMETRY_PRIORITY, &s_telemetry_task, TELEMETRY_CORE);
    app_state_set_telemetry_running(task_result == pdPASS);
    ESP_LOGI(TAG, "[TASK] Telemetry: %s", (task_result == pdPASS) ? "OK" : "FALHA");

    (void)flight_control_start();

    ESP_LOGI(TAG, "===== SISTEMA PRONTO =====");
    ESP_LOGI(TAG, "Rede: %s | Senha: %s | http://192.168.4.1",
             WIFI_AP_SSID, WIFI_AP_PASSWORD);

    /* O httpd e as tarefas tocam o sistema; o laco principal fica ocioso. */
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
