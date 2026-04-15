#include "tlc5947.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TLC5947_SPI_HOST SPI3_HOST
#define TLC5947_PIN_SCLK 3
#define TLC5947_PIN_XLAT 11
#define TLC5947_PIN_BLANK 10
#define TLC5947_PIN_SIN 6

#define TLC5947_SPI_CLOCK_HZ 12000000
#define TLC5947_BITS_PER_CHIP (TLC5947_CHANNELS_PER_CHIP * 12)
#define TLC5947_TOTAL_BITS (TLC5947_CHAIN_COUNT * TLC5947_BITS_PER_CHIP)
#define TLC5947_FRAME_BYTES (TLC5947_TOTAL_BITS / 8)

#define TLC5947_RGB_R 0
#define TLC5947_RGB_G 1
#define TLC5947_RGB_B 2
#define TLC5947_RGB8_MAX 255U
#define TLC5947_PWM_MAX 1024U
#define TLC5947_GAMMA 4.2f
#define TLC5947_IMAGE_FLIP_VERTICAL 1

static const char *TAG = "tlc5947";
static spi_device_handle_t s_tlc_dev = NULL;

static uint16_t s_gray[TLC5947_TOTAL_CHANNELS];
static uint8_t s_led_rgb[TLC5947_LED_COUNT][3];
static uint8_t s_pov_rgb[TLC5947_POV_COLUMNS][TLC5947_LED_COUNT][3];
static uint8_t s_frame[TLC5947_FRAME_BYTES];
static uint16_t s_gamma_lut[TLC5947_RGB8_MAX + 1U];
static uint16_t s_rainbow_phase = 0;
static uint32_t s_last_flush_time_us = 0;
static uint16_t s_r_channel_map[TLC5947_LED_COUNT];
static uint16_t s_g_channel_map[TLC5947_LED_COUNT];
static uint16_t s_b_channel_map[TLC5947_LED_COUNT];
static bool s_channel_map_ready = false;

static inline uint16_t tlc5947_clip_12bit(uint16_t v)
{
    return (v > 4095U) ? 4095U : v;
}

static inline uint16_t tlc5947_rgb8_to_pwm(uint8_t v)
{
    return s_gamma_lut[v];
}

static void tlc5947_build_gamma_lut(void)
{
    s_gamma_lut[0] = 0;
    for (uint16_t i = 1; i <= TLC5947_RGB8_MAX; ++i) {
        const float x = (float)i / (float)TLC5947_RGB8_MAX;
        const float y = powf(x, TLC5947_GAMMA);
        uint32_t pwm = (uint32_t)(y * (float)TLC5947_PWM_MAX + 0.5f);
        if (pwm > TLC5947_PWM_MAX) {
            pwm = TLC5947_PWM_MAX;
        }
        s_gamma_lut[i] = (uint16_t)pwm;
    }
    s_gamma_lut[TLC5947_RGB8_MAX] = TLC5947_PWM_MAX;
}

static inline uint16_t tlc5947_map_channel(uint16_t led_index, uint8_t color)
{
    const uint16_t chip_index = led_index / 8U;
    const uint16_t local_led = led_index % 8U;
    const uint16_t base = chip_index * TLC5947_CHANNELS_PER_CHIP;

    if (color == TLC5947_RGB_B) {
        return (uint16_t)(base + local_led);
    }

    const uint16_t reverse_led = (uint16_t)(7U - local_led);
    const uint16_t rg_base = (uint16_t)(8U + reverse_led * 2U);
    if (color == TLC5947_RGB_R) {
        return (uint16_t)(base + rg_base);
    }
    return (uint16_t)(base + rg_base + 1U);
}

static void tlc5947_build_channel_map_once(void)
{
    if (s_channel_map_ready) {
        return;
    }
    for (uint16_t led = 0; led < TLC5947_LED_COUNT; ++led) {
        s_r_channel_map[led] = tlc5947_map_channel(led, TLC5947_RGB_R);
        s_g_channel_map[led] = tlc5947_map_channel(led, TLC5947_RGB_G);
        s_b_channel_map[led] = tlc5947_map_channel(led, TLC5947_RGB_B);
    }
    s_channel_map_ready = true;
}

static void tlc5947_pack_frame(void)
{
    uint8_t *out = s_frame;
    for (int ch = TLC5947_TOTAL_CHANNELS - 1; ch >= 1; ch -= 2) {
        const uint16_t high = (uint16_t)(s_gray[ch] & 0x0FFFU);
        const uint16_t low = (uint16_t)(s_gray[ch - 1] & 0x0FFFU);
        *out++ = (uint8_t)(high >> 4);
        *out++ = (uint8_t)(((high & 0x000FU) << 4) | (low >> 8));
        *out++ = (uint8_t)(low & 0x00FFU);
    }
}

static inline void tlc5947_latch(void)
{
    gpio_set_level(TLC5947_PIN_BLANK, 1);
    gpio_set_level(TLC5947_PIN_XLAT, 1);
    gpio_set_level(TLC5947_PIN_XLAT, 0);
    gpio_set_level(TLC5947_PIN_BLANK, 0);
}

static void tlc5947_hue_to_rgb8(uint16_t hue, uint8_t max_rgb, uint8_t *r, uint8_t *g, uint8_t *b)
{
    const uint16_t h = (uint16_t)(hue % 1536U);
    const uint16_t seg = (uint16_t)(h / 256U);
    const uint16_t x = (uint16_t)(h % 256U);
    const uint32_t v = max_rgb;
    const uint32_t rising = (v * x) / 255U;
    const uint32_t falling = (v * (255U - x)) / 255U;

    uint8_t rr = 0;
    uint8_t gg = 0;
    uint8_t bb = 0;

    switch (seg) {
    case 0: rr = (uint8_t)v; gg = (uint8_t)rising; bb = 0; break;
    case 1: rr = (uint8_t)falling; gg = (uint8_t)v; bb = 0; break;
    case 2: rr = 0; gg = (uint8_t)v; bb = (uint8_t)rising; break;
    case 3: rr = 0; gg = (uint8_t)falling; bb = (uint8_t)v; break;
    case 4: rr = (uint8_t)rising; gg = 0; bb = (uint8_t)v; break;
    default: rr = (uint8_t)v; gg = 0; bb = (uint8_t)falling; break;
    }

    *r = rr;
    *g = gg;
    *b = bb;
}

esp_err_t tlc5947_init(void)
{
    if (s_tlc_dev != NULL) {
        return ESP_OK;
    }

    const spi_bus_config_t bus_cfg = {
        .mosi_io_num = TLC5947_PIN_SIN,
        .miso_io_num = -1,
        .sclk_io_num = TLC5947_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TLC5947_FRAME_BYTES,
    };

    const spi_device_interface_config_t dev_cfg = {
        .mode = 0,
        .clock_speed_hz = TLC5947_SPI_CLOCK_HZ,
        .spics_io_num = -1,
        .queue_size = 2,
    };

    esp_err_t ret = spi_bus_initialize(TLC5947_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    ret = spi_bus_add_device(TLC5947_SPI_HOST, &dev_cfg, &s_tlc_dev);
    if (ret == ESP_ERR_INVALID_STATE) {
        ret = ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "spi add device failed");

    const gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << TLC5947_PIN_XLAT) | (1ULL << TLC5947_PIN_BLANK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_cfg), TAG, "gpio config failed");

    gpio_set_level(TLC5947_PIN_XLAT, 0);
    gpio_set_level(TLC5947_PIN_BLANK, 0);

    tlc5947_build_gamma_lut();
    tlc5947_build_channel_map_once();
    tlc5947_clear_rgb_buffer();
    tlc5947_fill_pov_test_pattern();
    ESP_RETURN_ON_ERROR(tlc5947_update_from_rgb_buffer(), TAG, "initial frame flush failed");

    ESP_LOGI(TAG, "TLC5947 ready: leds=%d columns=%d spi=%dHz nominal=%uus",
             TLC5947_LED_COUNT, TLC5947_POV_COLUMNS, TLC5947_SPI_CLOCK_HZ, tlc5947_get_nominal_shift_time_us());
    return ESP_OK;
}

esp_err_t tlc5947_set_channel(uint16_t channel, uint16_t gray12)
{
    ESP_RETURN_ON_FALSE(channel < TLC5947_TOTAL_CHANNELS, ESP_ERR_INVALID_ARG, TAG, "channel out of range");
    s_gray[channel] = tlc5947_clip_12bit(gray12);
    return ESP_OK;
}

void tlc5947_set_all(uint16_t gray12)
{
    const uint16_t clipped = tlc5947_clip_12bit(gray12);
    for (uint16_t i = 0; i < TLC5947_TOTAL_CHANNELS; ++i) {
        s_gray[i] = clipped;
    }
}

esp_err_t tlc5947_flush(void)
{
    ESP_RETURN_ON_FALSE(s_tlc_dev != NULL, ESP_ERR_INVALID_STATE, TAG, "device not initialized");

    tlc5947_pack_frame();

    spi_transaction_t t = {
        .length = TLC5947_TOTAL_BITS,
        .tx_buffer = s_frame,
    };

    const int64_t t0 = esp_timer_get_time();
    esp_err_t ret = spi_device_polling_transmit(s_tlc_dev, &t);
    ESP_RETURN_ON_ERROR(ret, TAG, "spi transmit failed, err=%s", esp_err_to_name(ret));
    tlc5947_latch();
    s_last_flush_time_us = (uint32_t)(esp_timer_get_time() - t0);
    return ESP_OK;
}

uint16_t tlc5947_get_channel_count(void)
{
    return TLC5947_TOTAL_CHANNELS;
}

void tlc5947_clear_rgb_buffer(void)
{
    memset(s_led_rgb, 0, sizeof(s_led_rgb));
}

esp_err_t tlc5947_set_led_rgb(uint16_t led_index, uint8_t red, uint8_t green, uint8_t blue)
{
    ESP_RETURN_ON_FALSE(led_index < TLC5947_LED_COUNT, ESP_ERR_INVALID_ARG, TAG, "led index out of range");
    s_led_rgb[led_index][TLC5947_RGB_R] = red;
    s_led_rgb[led_index][TLC5947_RGB_G] = green;
    s_led_rgb[led_index][TLC5947_RGB_B] = blue;
    return ESP_OK;
}

esp_err_t tlc5947_update_from_rgb_buffer(void)
{
    tlc5947_build_channel_map_once();
    for (uint16_t led = 0; led < TLC5947_LED_COUNT; ++led) {
        s_gray[s_r_channel_map[led]] = tlc5947_rgb8_to_pwm(s_led_rgb[led][TLC5947_RGB_R]);
        s_gray[s_g_channel_map[led]] = tlc5947_rgb8_to_pwm(s_led_rgb[led][TLC5947_RGB_G]);
        s_gray[s_b_channel_map[led]] = tlc5947_rgb8_to_pwm(s_led_rgb[led][TLC5947_RGB_B]);
    }
    return tlc5947_flush();
}

esp_err_t tlc5947_rainbow_step(uint16_t phase_step, uint8_t max_rgb)
{
    for (uint16_t led = 0; led < TLC5947_LED_COUNT; ++led) {
        const uint16_t hue = (uint16_t)((s_rainbow_phase + (led * 1536U) / TLC5947_LED_COUNT) % 1536U);
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        tlc5947_hue_to_rgb8(hue, max_rgb, &r, &g, &b);
        s_led_rgb[led][TLC5947_RGB_R] = r;
        s_led_rgb[led][TLC5947_RGB_G] = g;
        s_led_rgb[led][TLC5947_RGB_B] = b;
    }

    s_rainbow_phase = (uint16_t)((s_rainbow_phase + phase_step) % 1536U);
    return tlc5947_update_from_rgb_buffer();
}

void tlc5947_clear_pov_buffer(void)
{
    memset(s_pov_rgb, 0, sizeof(s_pov_rgb));
}

esp_err_t tlc5947_set_pov_pixel(uint16_t column, uint16_t led_index, uint8_t red, uint8_t green, uint8_t blue)
{
    ESP_RETURN_ON_FALSE(column < TLC5947_POV_COLUMNS, ESP_ERR_INVALID_ARG, TAG, "column out of range");
    ESP_RETURN_ON_FALSE(led_index < TLC5947_LED_COUNT, ESP_ERR_INVALID_ARG, TAG, "led out of range");

    s_pov_rgb[column][led_index][TLC5947_RGB_R] = red;
    s_pov_rgb[column][led_index][TLC5947_RGB_G] = green;
    s_pov_rgb[column][led_index][TLC5947_RGB_B] = blue;
    return ESP_OK;
}

void tlc5947_fill_pov_test_pattern(void)
{
    tlc5947_clear_pov_buffer();
}

esp_err_t tlc5947_update_from_pov_column(uint16_t column)
{
    ESP_RETURN_ON_FALSE(column < TLC5947_POV_COLUMNS, ESP_ERR_INVALID_ARG, TAG, "column out of range");

    tlc5947_build_channel_map_once();
    for (uint16_t led = 0; led < TLC5947_LED_COUNT; ++led) {
        s_gray[s_r_channel_map[led]] = tlc5947_rgb8_to_pwm(s_pov_rgb[column][led][TLC5947_RGB_R]);
        s_gray[s_g_channel_map[led]] = tlc5947_rgb8_to_pwm(s_pov_rgb[column][led][TLC5947_RGB_G]);
        s_gray[s_b_channel_map[led]] = tlc5947_rgb8_to_pwm(s_pov_rgb[column][led][TLC5947_RGB_B]);
    }
    return tlc5947_flush();
}

esp_err_t tlc5947_load_pov_rgb_frame(const uint8_t *rgb_frame, size_t len)
{
    ESP_RETURN_ON_FALSE(rgb_frame != NULL, ESP_ERR_INVALID_ARG, TAG, "rgb_frame is null");
    ESP_RETURN_ON_FALSE(len == (size_t)(TLC5947_POV_COLUMNS * TLC5947_LED_COUNT * 3),
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "rgb frame size invalid");

    for (uint16_t row = 0; row < TLC5947_LED_COUNT; ++row) {
        const size_t row_base = (size_t)row * TLC5947_POV_COLUMNS * 3;
#if TLC5947_IMAGE_FLIP_VERTICAL
        const uint16_t dst_row = (uint16_t)(TLC5947_LED_COUNT - 1U - row);
#else
        const uint16_t dst_row = row;
#endif
        for (uint16_t col = 0; col < TLC5947_POV_COLUMNS; ++col) {
            const size_t idx = row_base + (size_t)col * 3;
            s_pov_rgb[col][dst_row][TLC5947_RGB_R] = rgb_frame[idx + 0];
            s_pov_rgb[col][dst_row][TLC5947_RGB_G] = rgb_frame[idx + 1];
            s_pov_rgb[col][dst_row][TLC5947_RGB_B] = rgb_frame[idx + 2];
        }
    }
    return ESP_OK;
}

uint32_t tlc5947_get_last_flush_time_us(void)
{
    return s_last_flush_time_us;
}

uint32_t tlc5947_get_nominal_shift_time_us(void)
{
    return (uint32_t)((TLC5947_TOTAL_BITS * 1000000ULL) / TLC5947_SPI_CLOCK_HZ);
}
