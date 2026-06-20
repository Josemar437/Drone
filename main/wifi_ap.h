/**
 * @file wifi_ap.h
 * @brief Sobe o Access Point Wi-Fi que hospeda a interface de controle.
 *
 * O drone nao se conecta a uma rede existente: ele cria a propria rede (AP) para
 * que o operador conecte o celular/PC diretamente e acesse o console web. SSID e
 * senha estao em config.h. O IP padrao do softAP do IDF (192.168.4.1) ja e o
 * esperado pela interface, entao nao e preciso reconfigurar o netif.
 */
#ifndef WIFI_AP_H
#define WIFI_AP_H

#include "esp_err.h"

/**
 * @brief Inicializa a pilha de rede e ativa o Access Point.
 *
 * Deve ser chamada apos a NVS estar inicializada (o Wi-Fi guarda calibracao de
 * radio na NVS). Se a senha tiver menos de 8 caracteres, o AP sobe aberto.
 *
 * @return ESP_OK se o AP foi ativado; codigo de erro em qualquer etapa que falhe.
 */
esp_err_t wifi_ap_start(void);

#endif /* WIFI_AP_H */
