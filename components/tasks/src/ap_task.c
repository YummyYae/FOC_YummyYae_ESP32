#include "ap_task.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "foc_control.h"
#include "tlc5947.h"

#define AP_TASK_CORE 0
#define AP_TASK_STACK_SIZE 6144
#define AP_TASK_PRIORITY 7

#define AP_WIFI_SSID "FOC_LED_AP"
#define AP_WIFI_PASS "12345678"
#define AP_WIFI_CHANNEL 6
#define AP_WIFI_MAX_CONN 2

#define AP_TCP_PORT 5005
#define AP_FRAME_HEADER_SIZE 8

#define AP_MAGIC_IMAGE "TCP1"
#define AP_MAGIC_SET_RPM "RPM1"
#define AP_MAGIC_TELEM "TEL1"
#define AP_MAGIC_SET_GAMMA "GAM1"

#define AP_IMG_FRAME_BYTES (TLC5947_POV_COLUMNS * TLC5947_LED_COUNT * 3)
#define AP_TELEM_PERIOD_US 200000

static const char *TAG = "ap_task";
static TaskHandle_t s_ap_task = NULL;
static uint8_t s_img_frame[AP_IMG_FRAME_BYTES];

static inline uint16_t rd_u16_le(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline void wr_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

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

static bool send_exact(int sock, const uint8_t *src, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        int n = send(sock, src + sent, len - sent, 0);
        if (n <= 0) {
            return false;
        }
        sent += (size_t)n;
    }
    return true;
}

static bool send_telem_frame(int client, uint16_t seq)
{
    uint8_t hdr[AP_FRAME_HEADER_SIZE];
    float payload[5];

    memcpy(hdr, AP_MAGIC_TELEM, 4);
    wr_u16_le(&hdr[4], seq);
    wr_u16_le(&hdr[6], (uint16_t)sizeof(payload));

    payload[0] = foc_params.mechanical_rpm;
    payload[1] = foc_params.target_mechanical_rpm;
    payload[2] = foc_params.uq;
    payload[3] = ((float)foc_params.shaft_angle_u16 * TWO_PI) / (float)FOC_ANGLE_FULL_TURN;
    payload[4] = ((float)foc_params.electrical_angle_u16 * TWO_PI) / (float)FOC_ANGLE_FULL_TURN;

    return send_exact(client, hdr, sizeof(hdr)) && send_exact(client, (const uint8_t *)payload, sizeof(payload));
}

static bool discard_payload(int client, uint16_t len)
{
    uint8_t drop[128];
    uint16_t remain = len;
    while (remain > 0) {
        uint16_t chunk = (remain > sizeof(drop)) ? (uint16_t)sizeof(drop) : remain;
        if (!recv_exact(client, drop, chunk)) {
            return false;
        }
        remain = (uint16_t)(remain - chunk);
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

        uint16_t telem_seq = 0;
        int64_t last_telem_us = esp_timer_get_time();

        while (true) {
            const int64_t now_us = esp_timer_get_time();
            if ((now_us - last_telem_us) >= AP_TELEM_PERIOD_US) {
                if (!send_telem_frame(client, telem_seq++)) {
                    break;
                }
                last_telem_us = now_us;
            }

            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(client, &readfds);
            struct timeval tv = {
                .tv_sec = 0,
                .tv_usec = 20000,
            };

            int sel = select(client + 1, &readfds, NULL, NULL, &tv);
            if (sel < 0) {
                break;
            }
            if (sel == 0 || !FD_ISSET(client, &readfds)) {
                continue;
            }

            if (!recv_exact(client, hdr, sizeof(hdr))) {
                break;
            }

            const uint16_t frame_id = rd_u16_le(&hdr[4]);
            const uint16_t payload_len = rd_u16_le(&hdr[6]);

            if (memcmp(hdr, AP_MAGIC_IMAGE, 4) == 0) {
                if (payload_len != AP_IMG_FRAME_BYTES) {
                    ESP_LOGW(TAG, "invalid image len=%u", (unsigned)payload_len);
                    break;
                }
                if (!recv_exact(client, s_img_frame, AP_IMG_FRAME_BYTES)) {
                    break;
                }
                if (tlc5947_load_pov_rgb_frame(s_img_frame, AP_IMG_FRAME_BYTES) != ESP_OK) {
                    ESP_LOGW(TAG, "image frame apply failed: id=%u", (unsigned)frame_id);
                }
            } else if (memcmp(hdr, AP_MAGIC_SET_RPM, 4) == 0) {
                if (payload_len != sizeof(float)) {
                    if (!discard_payload(client, payload_len)) {
                        break;
                    }
                    continue;
                }
                float new_target = 0.0f;
                if (!recv_exact(client, (uint8_t *)&new_target, sizeof(new_target))) {
                    break;
                }
                foc_params.target_mechanical_rpm = new_target;
            } else if (memcmp(hdr, AP_MAGIC_SET_GAMMA, 4) == 0) {
                if (payload_len != sizeof(float)) {
                    if (!discard_payload(client, payload_len)) {
                        break;
                    }
                    continue;
                }
                float new_gamma = 0.0f;
                if (!recv_exact(client, (uint8_t *)&new_gamma, sizeof(new_gamma))) {
                    break;
                }
                tlc5947_set_gamma(new_gamma);
            } else {
                if (!discard_payload(client, payload_len)) {
                    break;
                }
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
