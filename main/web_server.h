/**
 * @file web_server.h
 * @brief Servidor HTTP que serve a interface e expoe as rotas de controle/telemetria.
 *
 * Roda sobre esp_http_server, que cria a propria tarefa interna para atender as
 * conexoes. As rotas expoem codigos de status e formatos JSON compativeis com a
 * pagina web embutida.
 *
 * Deve ser iniciado apos os modulos que ele orquestra (esc_pwm, flight_control,
 * sensor_hub) ja estarem prontos.
 */
#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"
#include "sensor_hub.h"

/**
 * @brief Inicia o servidor HTTP e registra todas as rotas.
 *
 * @param hub Agregador de sensores, usado pelas rotas /sensors e /flightStatus.
 * @return ESP_OK se o servidor subiu; codigo de erro caso contrario.
 */
esp_err_t web_server_start(sensor_hub_t *hub);

/**
 * @brief Indica se o servidor esta no ar (para a rota /health).
 * @return true se iniciado.
 */
bool web_server_running(void);

#endif /* WEB_SERVER_H */
