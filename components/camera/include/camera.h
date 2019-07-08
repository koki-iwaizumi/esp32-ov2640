#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "rom/lldesc.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/ledc.h"

typedef union {
    struct {
        uint8_t sample2;
        uint8_t unused2;
        uint8_t sample1;
        uint8_t unused1;
    };
    uint32_t val;
} dma_elem_t;

typedef enum {
    SM_0A0B_0B0C = 0,
    SM_0A0B_0C0D = 1,
    SM_0A00_0B00 = 3,
} i2s_sampling_mode_t;

typedef struct {
    int pin_reset;
    int pin_xclk;
    int pin_sscb_sda;
    int pin_sscb_scl;
    int pin_d7;
    int pin_d6;
    int pin_d5;
    int pin_d4;
    int pin_d3;
    int pin_d2;
    int pin_d1;
    int pin_d0;
    int pin_vsync;
    int pin_href;
    int pin_pclk;

    int xclk_freq_hz;

    ledc_timer_t ledc_timer;
    ledc_channel_t ledc_channel;

    size_t width;
    size_t height;

} camera_config_t;

typedef void (*dma_filter_t)(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);

typedef struct {
    camera_config_t config;
    uint8_t *fb;
    size_t fb_size;
    size_t data_size;
    size_t in_bytes_per_pixel;
    size_t fb_bytes_per_pixel;
    size_t stride;

    lldesc_t *dma_desc;
    dma_elem_t **dma_buf;
    bool dma_done;
    size_t dma_desc_count;
    size_t dma_desc_cur;
    size_t dma_received_count;
    size_t dma_filtered_count;
    size_t dma_per_line;
    size_t dma_buf_width;
    size_t dma_sample_count;
    i2s_sampling_mode_t sampling_mode;
    dma_filter_t dma_filter;
    intr_handle_t i2s_intr_handle;
    QueueHandle_t data_ready;
    SemaphoreHandle_t frame_ready;
    TaskHandle_t dma_filter_task;
} camera_state_t;


esp_err_t camera_init(const camera_config_t* config);
esp_err_t camera_deinit();
uint8_t* camera_get_fb();
size_t camera_get_data_size();
int camera_get_fb_width();
int camera_get_fb_height();
esp_err_t camera_run();
void camera_print_fb();
