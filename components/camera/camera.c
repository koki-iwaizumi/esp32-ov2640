#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "time.h"
#include "sys/time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/lldesc.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/periph_ctrl.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "camera.h"

#define ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE 0x20000

static const char* TAG = "camera_demo camera";

camera_state_t* s_state = NULL;

static void i2s_init();
static void i2s_run();
static void IRAM_ATTR gpio_isr(void* arg);
static void IRAM_ATTR i2s_isr(void* arg);
static esp_err_t dma_desc_init();
static void dma_desc_deinit();
static void dma_filter_task(void *pvParameters);
static void dma_filter_jpeg(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst);
static void i2s_stop();

static bool is_hs_mode(){
    return s_state->config.xclk_freq_hz > 10000000;
}

static size_t i2s_bytes_per_sample(i2s_sampling_mode_t mode){
    switch(mode) {
        case SM_0A00_0B00:
            return 4;
        case SM_0A0B_0B0C:
            return 4;
        case SM_0A0B_0C0D:
            return 2;
        default:
            assert(0 && "invalid sampling mode");
            return 0;
    }
}

static void vsync_intr_disable(){
    gpio_set_intr_type(s_state->config.pin_vsync, GPIO_INTR_DISABLE);
}

static void vsync_intr_enable(){
    gpio_set_intr_type(s_state->config.pin_vsync, GPIO_INTR_NEGEDGE);
}


esp_err_t camera_init(const camera_config_t* config){

    esp_err_t err = ESP_OK;

    s_state = (camera_state_t*) calloc(sizeof(*s_state), 1);
    if( ! s_state){
        return ESP_ERR_NO_MEM;
    }

    memcpy(&s_state->config, config, sizeof(*config));


	int compression_ratio_bound = 10;
	size_t equiv_line_count = s_state->config.height / compression_ratio_bound;
	s_state->fb_size = s_state->config.width * equiv_line_count * 2;
	s_state->dma_filter = &dma_filter_jpeg;

	if(is_hs_mode()){
		s_state->sampling_mode = SM_0A0B_0B0C;
	} else {
		s_state->sampling_mode = SM_0A00_0B00;
	}

	s_state->in_bytes_per_pixel = 2;
	s_state->fb_bytes_per_pixel = 2;

    ESP_LOGI(TAG, "Setting frame size to %dx%d", s_state->config.width, s_state->config.height);
    ESP_LOGI(TAG, "in_bpp: %d, fb_bpp: %d, fb_size: %d, mode: %d, width: %d height: %d", s_state->in_bytes_per_pixel, s_state->fb_bytes_per_pixel, s_state->fb_size, s_state->sampling_mode, s_state->config.width, s_state->config.height);

	//heap memory confirm
	ESP_LOGI(TAG, "esp_get_free_heap_size (%d bytes)", esp_get_free_heap_size());
	ESP_LOGI(TAG, "esp_get_minimum_free_heap_size (%d bytes)", esp_get_minimum_free_heap_size());

	ESP_LOGI(TAG, "heap_caps_get_free_size(MALLOC_CAP_8BIT) (%d bytes)", heap_caps_get_free_size(MALLOC_CAP_8BIT));
	ESP_LOGI(TAG, "heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT) (%d bytes)", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
	ESP_LOGI(TAG, "heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) (%d bytes)", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    ESP_LOGI(TAG, "Allocating frame buffer (%d bytes)", s_state->fb_size);
    s_state->fb = (uint8_t*) calloc(s_state->fb_size, 1);
    if(s_state->fb == NULL){
        ESP_LOGE(TAG, "Failed to allocate frame buffer");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    ESP_LOGI(TAG, "Initializing I2S and DMA");
    i2s_init();
    err = dma_desc_init();
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Failed to initialize I2S and DMA");
        goto fail;
    }

    s_state->data_ready = xQueueCreate(16, sizeof(size_t));
    s_state->frame_ready = xSemaphoreCreateBinary();
    if(s_state->data_ready == NULL || s_state->frame_ready == NULL){
        ESP_LOGE(TAG, "Failed to create semaphores");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    if( ! xTaskCreatePinnedToCore(&dma_filter_task, "dma_filter", 4096, NULL, 10, &s_state->dma_filter_task, 1)){
        ESP_LOGE(TAG, "Failed to create DMA filter task");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    ESP_LOGI(TAG, "Initializing GPIO interrupts");
    vsync_intr_disable();
    err = gpio_isr_handler_add(s_state->config.pin_vsync, &gpio_isr, NULL);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "gpio_isr_handler_add failed (%x)", err);
        goto fail;
    }

    // skip at least one frame after changing camera settings
    while(gpio_get_level(s_state->config.pin_vsync) == 0){}
    while(gpio_get_level(s_state->config.pin_vsync) != 0){}
    while(gpio_get_level(s_state->config.pin_vsync) == 0){}

    ESP_LOGI(TAG, "Init done");
    return ESP_OK;

fail:
    camera_deinit();
    return err;
}

esp_err_t camera_deinit(){
    if(s_state->dma_filter_task){
        vTaskDelete(s_state->dma_filter_task);
    }
    if(s_state->data_ready){
        vQueueDelete(s_state->data_ready);
    }
    if(s_state->frame_ready){
        vSemaphoreDelete(s_state->frame_ready);
    }
    gpio_isr_handler_remove(s_state->config.pin_vsync);
    if(s_state->i2s_intr_handle){
        esp_intr_disable(s_state->i2s_intr_handle);
        esp_intr_free(s_state->i2s_intr_handle);
    }
    dma_desc_deinit();
    free(s_state->fb);
    free(s_state);
    s_state = NULL;
	periph_module_disable(PERIPH_LEDC_MODULE);
    periph_module_disable(PERIPH_I2S0_MODULE);
    return ESP_OK;
}

uint8_t* camera_get_fb(){
    return s_state->fb;
}

int camera_get_fb_width(){
    return s_state->config.width;
}

int camera_get_fb_height(){
    return s_state->config.height;
}

size_t camera_get_data_size(){
    return s_state->data_size;
}

esp_err_t camera_run(){
    memset(s_state->fb, 0, s_state->fb_size);
    i2s_run();
    ESP_LOGI(TAG, "Waiting for frame");
    xSemaphoreTake(s_state->frame_ready, portMAX_DELAY);
    return ESP_OK;
}

static esp_err_t dma_desc_init(){
    assert(s_state->config.width % 4 == 0);
    size_t line_size = s_state->config.width * s_state->in_bytes_per_pixel * i2s_bytes_per_sample(s_state->sampling_mode);
    ESP_LOGI(TAG, "Line width (for DMA): %d bytes", line_size);
    size_t dma_per_line = 1;
    size_t buf_size = line_size;
    while(buf_size >= 4096){
        buf_size /= 2;
        dma_per_line *= 2;
    }
    size_t dma_desc_count = dma_per_line * 4;
    s_state->dma_buf_width = line_size;
    s_state->dma_per_line = dma_per_line;
    s_state->dma_desc_count = dma_desc_count;
    ESP_LOGI(TAG, "DMA buffer size: %d, DMA buffers per line: %d", buf_size, dma_per_line);
    ESP_LOGI(TAG, "DMA buffer count: %d", dma_desc_count);

    s_state->dma_buf = (dma_elem_t**) malloc(sizeof(dma_elem_t*) * dma_desc_count);
    if(s_state->dma_buf == NULL){
        return ESP_ERR_NO_MEM;
    }
    s_state->dma_desc = (lldesc_t*) malloc(sizeof(lldesc_t) * dma_desc_count);
    if(s_state->dma_desc == NULL){
        return ESP_ERR_NO_MEM;
    }
    size_t dma_sample_count = 0;
    for(int i = 0; i < dma_desc_count; ++i){
        ESP_LOGI(TAG, "Allocating DMA buffer #%d, size=%d", i, buf_size);
        dma_elem_t* buf = (dma_elem_t*) malloc(buf_size);
        if(buf == NULL){
            return ESP_ERR_NO_MEM;
        }
        s_state->dma_buf[i] = buf;
        ESP_LOGV(TAG, "dma_buf[%d]=%p", i, buf);

        lldesc_t* pd = &s_state->dma_desc[i];
        pd->length = buf_size;
        if(s_state->sampling_mode == SM_0A0B_0B0C && (i + 1) % dma_per_line == 0){
            pd->length -= 4;
        }
        dma_sample_count += pd->length / 4;
        pd->size = pd->length;
        pd->owner = 1;
        pd->sosf = 1;
        pd->buf = (uint8_t*) buf;
        pd->offset = 0;
        pd->empty = 0;
        pd->eof = 1;
        pd->qe.stqe_next = &s_state->dma_desc[(i + 1) % dma_desc_count];
    }
    s_state->dma_done = false;
    s_state->dma_sample_count = dma_sample_count;
    return ESP_OK;
}

static void dma_desc_deinit(){
    if(s_state->dma_buf){
        for (int i = 0; i < s_state->dma_desc_count; ++i) {
            free(s_state->dma_buf[i]);
        }
    }
    free(s_state->dma_buf);
    free(s_state->dma_desc);
}

static inline void i2s_conf_reset(){
    const uint32_t lc_conf_reset_flags = I2S_IN_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
    I2S0.lc_conf.val |= lc_conf_reset_flags;
    I2S0.lc_conf.val &= ~lc_conf_reset_flags;

    const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
    I2S0.conf.val |= conf_reset_flags;
    I2S0.conf.val &= ~conf_reset_flags;
    while(I2S0.state.rx_fifo_reset_back){}
}

static void i2s_init(){
    camera_config_t* config = &s_state->config;

    // Configure input GPIOs
    gpio_num_t pins[] = {
            config->pin_d7,
            config->pin_d6,
            config->pin_d5,
            config->pin_d4,
            config->pin_d3,
            config->pin_d2,
            config->pin_d1,
            config->pin_d0,
            config->pin_vsync,
            config->pin_href,
            config->pin_pclk
    };
    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    for(int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i){
        if (rtc_gpio_is_valid_gpio(pins[i])) {
            rtc_gpio_deinit(pins[i]);
        }
        conf.pin_bit_mask = 1LL << pins[i];
        gpio_config(&conf);
    }

    // Route input GPIOs to I2S peripheral using GPIO matrix
    gpio_matrix_in(config->pin_d0, I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(config->pin_d1, I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(config->pin_d2, I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(config->pin_d3, I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(config->pin_d4, I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(config->pin_d5, I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(config->pin_d6, I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(config->pin_d7, I2S0I_DATA_IN7_IDX, false);
    gpio_matrix_in(config->pin_vsync, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(config->pin_href, I2S0I_H_ENABLE_IDX, false);
    gpio_matrix_in(config->pin_pclk, I2S0I_WS_IN_IDX, false);

    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);
    // Toggle some reset bits in LC_CONF register
    // Toggle some reset bits in CONF register
    i2s_conf_reset();
    // Enable slave mode (sampling clock is external)
    I2S0.conf.rx_slave_mod = 1;
    // Enable parallel mode
    I2S0.conf2.lcd_en = 1;
    // Use HSYNC/VSYNC/HREF to control sampling
    I2S0.conf2.camera_en = 1;
    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    // FIFO will sink data to DMA
    I2S0.fifo_conf.dscr_en = 1;
    // FIFO configuration
    I2S0.fifo_conf.rx_fifo_mod = s_state->sampling_mode;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.conf_chan.rx_chan_mod = 1;
    // Clear flags which are used in I2S serial mode
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;
    I2S0.timing.val = 0;

    // Allocate I2S interrupt, keep it disabled
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE,
    ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
            &i2s_isr, NULL, &s_state->i2s_intr_handle);
}


static void i2s_stop(){
    esp_intr_disable(s_state->i2s_intr_handle);
    vsync_intr_disable();
    i2s_conf_reset();
    I2S0.conf.rx_start = 0;
    size_t val = SIZE_MAX;
    BaseType_t higher_priority_task_woken;
    xQueueSendFromISR(s_state->data_ready, &val, &higher_priority_task_woken);
}

static void i2s_run(){
    for(int i = 0; i < s_state->dma_desc_count; ++i){
        lldesc_t* d = &s_state->dma_desc[i];
        ESP_LOGI(TAG, "DMA desc %2d: %u %u %u %u %u %u %p %p", i, d->length, d->size, d->offset, d->eof, d->sosf, d->owner, d->buf, d->qe.stqe_next);
        memset(s_state->dma_buf[i], 0, d->length);
    }

    // wait for vsync
    ESP_LOGI(TAG, "Waiting for positive edge on VSYNC");
    while(gpio_get_level(s_state->config.pin_vsync) == 0){}
    while(gpio_get_level(s_state->config.pin_vsync) != 0){}
    ESP_LOGI(TAG, "Got VSYNC");

    s_state->dma_done = false;
    s_state->dma_desc_cur = 0;
    s_state->dma_received_count = 0;
    s_state->dma_filtered_count = 0;
    esp_intr_disable(s_state->i2s_intr_handle);
    i2s_conf_reset();

    I2S0.rx_eof_num = s_state->dma_sample_count;
    I2S0.in_link.addr = (uint32_t) &s_state->dma_desc[0];
    I2S0.in_link.start = 1;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.int_ena.val = 0;
    I2S0.int_ena.in_done = 1;
    esp_intr_enable(s_state->i2s_intr_handle);
    vsync_intr_enable();
    I2S0.conf.rx_start = 1;

}

static void IRAM_ATTR signal_dma_buf_received(bool* need_yield){
    size_t dma_desc_filled = s_state->dma_desc_cur;
    s_state->dma_desc_cur = (dma_desc_filled + 1) % s_state->dma_desc_count;
    s_state->dma_received_count++;
    BaseType_t higher_priority_task_woken;
    BaseType_t ret = xQueueSendFromISR(s_state->data_ready, &dma_desc_filled, &higher_priority_task_woken);
    if (ret != pdTRUE) {
        ESP_EARLY_LOGW(TAG, "queue send failed (%d), dma_received_count=%d", ret, s_state->dma_received_count);
    }
    *need_yield = (ret == pdTRUE && higher_priority_task_woken == pdTRUE);
}

static void IRAM_ATTR i2s_isr(void* arg){
    I2S0.int_clr.val = I2S0.int_raw.val;
    bool need_yield;
    signal_dma_buf_received(&need_yield);
    ESP_EARLY_LOGV(TAG, "isr, cnt=%d", s_state->dma_received_count);
    if(s_state->dma_received_count == s_state->config.height * s_state->dma_per_line){
        i2s_stop();
    }
    if (need_yield) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR gpio_isr(void* arg){
    bool need_yield = false;
    ESP_EARLY_LOGV(TAG, "gpio isr, cnt=%d", s_state->dma_received_count);
    if(gpio_get_level(s_state->config.pin_vsync) == 0 && s_state->dma_received_count > 0 && ! s_state->dma_done){
        signal_dma_buf_received(&need_yield);
        i2s_stop();
    }
    if(need_yield){
        portYIELD_FROM_ISR();
    }
}

static size_t get_fb_pos(){
    return s_state->dma_filtered_count * s_state->config.width * s_state->fb_bytes_per_pixel / s_state->dma_per_line;
}

static void IRAM_ATTR dma_filter_task(void *pvParameters){
    while(true){
        size_t buf_idx;
        xQueueReceive(s_state->data_ready, &buf_idx, portMAX_DELAY);
        if(buf_idx == SIZE_MAX){
            s_state->data_size = get_fb_pos();
            xSemaphoreGive(s_state->frame_ready);
            continue;
        }

        size_t fb_pos = get_fb_pos();
        assert(fb_pos <= s_state->fb_size + s_state->config.width * s_state->fb_bytes_per_pixel / s_state->dma_per_line);

        uint8_t* pfb = s_state->fb + fb_pos;
        const dma_elem_t* buf = s_state->dma_buf[buf_idx];
        lldesc_t* desc = &s_state->dma_desc[buf_idx];
        (*s_state->dma_filter)(buf, desc, pfb);
        s_state->dma_filtered_count++;
        ESP_LOGI(TAG, "dma_flt: flt_count=%d ", s_state->dma_filtered_count);
    }
}

static void IRAM_ATTR dma_filter_jpeg(const dma_elem_t* src, lldesc_t* dma_desc, uint8_t* dst){
    assert(s_state->sampling_mode == SM_0A0B_0B0C || s_state->sampling_mode == SM_0A00_0B00 );
    size_t end = dma_desc->length / sizeof(dma_elem_t) / 4;
    // manually unrolling 4 iterations of the loop here
    for(size_t i = 0; i < end; ++i){
        dst[0] = src[0].sample1;
        dst[1] = src[1].sample1;
        dst[2] = src[2].sample1;
        dst[3] = src[3].sample1;
        src += 4;
        dst += 4;
    }
    // the final sample of a line in SM_0A0B_0B0C sampling mode needs special handling
    if((dma_desc->length & 0x7) != 0){
        dst[0] = src[0].sample1;
        dst[1] = src[1].sample1;
        dst[2] = src[2].sample1;
        dst[3] = src[2].sample2;
    }
}
