#ifndef TLC5947_H
#define TLC5947_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TLC5947_CHAIN_COUNT 4
#define TLC5947_CHANNELS_PER_CHIP 24
#define TLC5947_TOTAL_CHANNELS (TLC5947_CHAIN_COUNT * TLC5947_CHANNELS_PER_CHIP)
#define TLC5947_LED_COUNT (TLC5947_CHAIN_COUNT * 8)
#define TLC5947_POV_COLUMNS 192

esp_err_t tlc5947_init(void);
esp_err_t tlc5947_set_channel(uint16_t channel, uint16_t gray12);
void tlc5947_set_all(uint16_t gray12);
esp_err_t tlc5947_flush(void);
uint16_t tlc5947_get_channel_count(void);

void tlc5947_clear_rgb_buffer(void);
esp_err_t tlc5947_set_led_rgb(uint16_t led_index, uint8_t red, uint8_t green, uint8_t blue);
esp_err_t tlc5947_update_from_rgb_buffer(void);
esp_err_t tlc5947_rainbow_step(uint16_t phase_step, uint8_t max_rgb);
uint32_t tlc5947_get_last_flush_time_us(void);
uint32_t tlc5947_get_nominal_shift_time_us(void);

void tlc5947_clear_pov_buffer(void);
esp_err_t tlc5947_set_pov_pixel(uint16_t column, uint16_t led_index, uint8_t red, uint8_t green, uint8_t blue);
void tlc5947_fill_pov_test_pattern(void);
esp_err_t tlc5947_update_from_pov_column(uint16_t column);
esp_err_t tlc5947_load_pov_rgb_frame(const uint8_t *rgb_frame, size_t len);

#ifdef __cplusplus
}
#endif

#endif
