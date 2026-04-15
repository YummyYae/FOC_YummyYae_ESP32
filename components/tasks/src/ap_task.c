#include "ap_task.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
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
#include "tlc5947.h"

#define AP_TASK_CORE 0
#define AP_TASK_STACK_SIZE 6144
#define AP_TASK_PRIORITY 7

#define AP_WIFI_SSID "FOC_LED_AP"
#define AP_WIFI_PASS "12345678"
#define AP_WIFI_CHANNEL 6
#define AP_WIFI_MAX_CONN 2

#define AP_TCP_PORT 5005
#define AP_FRAME_MAGIC "TCP1"
#define AP_FRAME_HEADER_SIZE 8
#define AP_IMG_FRAME_BYTES (TLC5947_POV_COLUMNS * TLC5947_LED_COUNT * 3)

static const char *TAG = "ap_task";
static TaskHandle_t s_ap_task = NULL;
static uint8_t s_img_frame[AP_IMG_FRAME_BYTES];

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

    ESP_LOGI(TAG, "AP started: ssid=%s channel=%d", AP_WIFI_SSID, AP_WIFI_CHANNEL);
    return ESP_OK;
}

static bool recv_exact(int sock, uint8_t *dst, size_t len)
{
    size_t got = 0;
    while (got < len) {
        int n = recv(sock, dst + got, len - got, 0);
        if (n <= 0) {
            return false;
        }
        got += (size_t)n;
    }
    return true;
}

static void ap_tcp_server_loop(void)
{
    int server = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server < 0) {
        ESP_LOGE(TAG, "create tcp socket failed errno=%d", errno);
        return;
    }

    int reuse = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(AP_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(server, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "bind tcp %d failed errno=%d", AP_TCP_PORT, errno);
        close(server);
        return;
    }
    if (listen(server, 1) < 0) {
        ESP_LOGE(TAG, "listen failed errno=%d", errno);
        close(server);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", AP_TCP_PORT);

    uint8_t hdr[AP_FRAME_HEADER_SIZE];
    while (true) {
        struct sockaddr_in client_addr = {0};
        socklen_t addr_len = sizeof(client_addr);
        int client = accept(server, (struct sockaddr *)&client_addr, &addr_len);
        if (client < 0) {
            ESP_LOGW(TAG, "accept failed errno=%d", errno);
            continue;
        }
        ESP_LOGI(TAG, "tcp client connected");

        while (true) {
            if (!recv_exact(client, hdr, sizeof(hdr))) {
                break;
            }
            if (memcmp(hdr, AP_FRAME_MAGIC, 4) != 0) {
                ESP_LOGW(TAG, "invalid frame magic");
                break;
            }

            const uint16_t frame_id = (uint16_t)(hdr[4] | ((uint16_t)hdr[5] << 8));
            const uint16_t payload_len = (uint16_t)(hdr[6] | ((uint16_t)hdr[7] << 8));
            if (payload_len != AP_IMG_FRAME_BYTES) {
                ESP_LOGW(TAG, "invalid frame len=%u", (unsigned)payload_len);
                break;
            }

            if (!recv_exact(client, s_img_frame, AP_IMG_FRAME_BYTES)) {
                break;
            }

            if (tlc5947_load_pov_rgb_frame(s_img_frame, AP_IMG_FRAME_BYTES) == ESP_OK) {
                ESP_LOGI(TAG, "image frame applied: id=%u", (unsigned)frame_id);
            } else {
                ESP_LOGW(TAG, "image frame apply failed: id=%u", (unsigned)frame_id);
            }
        }

        shutdown(client, 0);
        close(client);
        ESP_LOGW(TAG, "tcp client disconnected");
    }
}

static void ap_task_entry(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(ap_wifi_init());
    ap_tcp_server_loop();
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

int ap_task_send_udp_text(const char *data, size_t len)
{
    (void)data;
    (void)len;
    return -1;
}
