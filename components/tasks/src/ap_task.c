#include "ap_task.h"

#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define AP_TASK_CORE 0
#define AP_TASK_STACK_SIZE 6144
#define AP_TASK_PRIORITY 3

#define AP_WIFI_SSID "FOC_LED_AP"
#define AP_WIFI_PASS "12345678"
#define AP_WIFI_CHANNEL 6
#define AP_WIFI_MAX_CONN 2

#define AP_UDP_PORT 5005
#define AP_UDP_RX_BUF 256

static const char *TAG = "ap_task";
static TaskHandle_t s_ap_task = NULL;

static esp_err_t ap_wifi_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "nvs init failed");

    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "set mode failed");

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_WIFI_SSID,
            .ssid_len = strlen(AP_WIFI_SSID),
            .password = AP_WIFI_PASS,
            .channel = AP_WIFI_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = AP_WIFI_MAX_CONN,
        },
    };
    if (strlen(AP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), TAG, "set ap config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");

    ESP_LOGI(TAG, "AP started: ssid=%s pass=%s channel=%d", AP_WIFI_SSID, AP_WIFI_PASS, AP_WIFI_CHANNEL);
    return ESP_OK;
}

static void ap_udp_server_loop(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "create socket failed");
        return;
    }

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(AP_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "bind udp %d failed", AP_UDP_PORT);
        close(sock);
        return;
    }

    ESP_LOGI(TAG, "UDP server listening on port %d", AP_UDP_PORT);

    struct sockaddr_in last_client = {0};
    socklen_t last_client_len = sizeof(last_client);
    bool has_client = false;
    uint32_t heartbeat = 0;
    char rx_buf[AP_UDP_RX_BUF];
    char tx_buf[AP_UDP_RX_BUF];

    while (true) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sock, &readfds);

        struct timeval tv = {
            .tv_sec = 0,
            .tv_usec = 100000,
        };

        int sel = select(sock + 1, &readfds, NULL, NULL, &tv);
        if (sel > 0 && FD_ISSET(sock, &readfds)) {
            struct sockaddr_in client_addr = {0};
            socklen_t addr_len = sizeof(client_addr);
            int len = recvfrom(sock, rx_buf, sizeof(rx_buf) - 1, 0, (struct sockaddr *)&client_addr, &addr_len);
            if (len > 0) {
                rx_buf[len] = '\0';
                last_client = client_addr;
                last_client_len = addr_len;
                has_client = true;

                snprintf(tx_buf, sizeof(tx_buf), "AP_ACK:%s", rx_buf);
                sendto(sock, tx_buf, strlen(tx_buf), 0, (struct sockaddr *)&client_addr, addr_len);
            }
        }

        if (has_client) {
            heartbeat++;
            if (heartbeat >= 10) {
                heartbeat = 0;
                snprintf(tx_buf, sizeof(tx_buf), "AP_HEARTBEAT:%lu", (unsigned long)xTaskGetTickCount());
                sendto(sock, tx_buf, strlen(tx_buf), 0, (struct sockaddr *)&last_client, last_client_len);
            }
        }
    }
}

static void ap_task_entry(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(ap_wifi_init());
    ap_udp_server_loop();
    vTaskDelete(NULL);
}

esp_err_t ap_task_start(void)
{
    if (s_ap_task != NULL) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(ap_task_entry,
                                            "ap_core0",
                                            AP_TASK_STACK_SIZE,
                                            NULL,
                                            AP_TASK_PRIORITY,
                                            &s_ap_task,
                                            AP_TASK_CORE);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "create ap task failed");
    ESP_LOGI(TAG, "AP task started on core%d", AP_TASK_CORE);
    return ESP_OK;
}
