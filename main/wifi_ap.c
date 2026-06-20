/**
 * @file wifi_ap.c
 * @brief Implementacao do Access Point Wi-Fi (ver wifi_ap.h).
 *
 * Segue a sequencia de inicializacao do esp_wifi: netif -> loop de eventos ->
 * driver Wi-Fi -> configuracao do AP -> start. O IP default do softAP do IDF
 * e 192.168.4.1.
 */
#include "wifi_ap.h"

#include <string.h>
#include "config.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"

#define WIFI_AP_CHANNEL         (1)
#define WIFI_AP_MAX_CONN        (4)

static const char *TAG = "wifi-ap";

esp_err_t wifi_ap_start(void)
{
    esp_err_t err;
    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t ap_config;

    ESP_LOGI(TAG, "Configurando Access Point: %s", WIFI_AP_SSID);

    err = esp_netif_init();
    if (err != ESP_OK)
    {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK)
    {
        return err;
    }
    (void)esp_netif_create_default_wifi_ap();

    err = esp_wifi_init(&init_config);
    if (err != ESP_OK)
    {
        return err;
    }

    (void)memset(&ap_config, 0, sizeof(ap_config));
    (void)strncpy((char *)ap_config.ap.ssid, WIFI_AP_SSID, sizeof(ap_config.ap.ssid) - 1U);
    ap_config.ap.ssid_len = (uint8_t)strlen(WIFI_AP_SSID);
    (void)strncpy((char *)ap_config.ap.password, WIFI_AP_PASSWORD, sizeof(ap_config.ap.password) - 1U);
    ap_config.ap.channel = WIFI_AP_CHANNEL;
    ap_config.ap.max_connection = WIFI_AP_MAX_CONN;
    ap_config.ap.authmode = (strlen(WIFI_AP_PASSWORD) >= 8U) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    /* Forca WPA2-CCMP (AES). Sem isto o WPA2 pode negociar TKIP, que dispositivos
     * modernos rejeitam: a estacao chega a associar (join), mas o handshake de
     * 4 vias falha, nenhum dado trafega e o cliente nao obtem IP (fica em
     * "obtendo IP" e desconecta). */
    ap_config.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK)
    {
        return err;
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (err != ESP_OK)
    {
        return err;
    }
    err = esp_wifi_start();
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "AP ativo. IP: 192.168.4.1");
    }
    return err;
}
