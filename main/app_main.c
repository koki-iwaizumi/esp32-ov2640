#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "camera.h"
#include "http_server.h"

/********* config ***********/
#define WIFI_SSID "*"
#define WIFI_PASSWORD "*"

/********* i2c ***********/
#define I2C_CAMERA_TX_BUF_DISABLE 0
#define I2C_CAMERA_RX_BUF_DISABLE 0
#define I2C_CAMERA_FREQ_HZ 100000
#define I2C_CAMERA_NUM I2C_NUM_0
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

#define CAMERA_SLAVE_ADDRESS_READ 0b01100001
#define CAMERA_SLAVE_ADDRESS_WRITE 0b01100000

/********* gpio ***********/
#define PIN_D0 35
#define PIN_D1 17
#define PIN_D2 34
#define PIN_D3 5
#define PIN_D4 39
#define PIN_D5 18
#define PIN_D6 36
#define PIN_D7 19
#define PIN_XCLK 27
#define PIN_PCLK 21
#define PIN_VSYNC 22
#define PIN_HREF 26
#define PIN_SDA 25
#define PIN_SCL 23
#define PIN_RESET 15

#define GPIO_OUTPUT_PIN_SEL  (1 << PIN_RESET)

static const char* TAG = "camera_demo app_main";

static EventGroupHandle_t s_wifi_event_group;
const int CONNECTED_BIT = BIT0;
static ip4_addr_t s_ip_addr;

/********* camera reg ***********/
static const uint8_t default_regs[][2] = {
	{0xFF, 0x00},
	{0x2C, 0xFF},
	{0x2E, 0xDF},

	{0xFF, 0x01},
	{0x3C, 0x32},
	{0x11, 0x80},
	{0x09, 0x02},
	{0x28, 0x00},
	{0x13, 0xE5},
	{0x14, 0x48},
	{0x15, 0x00},
	{0x2C, 0x0C},
	{0x33, 0x78},
	{0x3A, 0x33},
	{0x3B, 0xFB},
	{0x3E, 0x00},
	{0x43, 0x11},
	{0x16, 0x10},
	{0x39, 0x02},
	{0x35, 0x88},
	{0x22, 0x0A},
	{0x37, 0x40},
	{0x23, 0x00},
	{0x34, 0xA0},
	{0x06, 0x02},
	{0x06, 0x88},
	{0x07, 0xC0},
	{0x0D, 0xB7},
	{0x0E, 0x01},
	{0x4C, 0x00},
	{0x4A, 0x81},
	{0x21, 0x99},
	{0x24, 0x40},
	{0x25, 0x38},
	{0x26, 0x82},
	{0x48, 0x00},
	{0x49, 0x00},
	{0x5C, 0x00},
	{0x63, 0x00},
	{0x46, 0x00},
	{0x47, 0x00},
	{0x0C, 0x3A},
	{0x5D, 0x55},
	{0x5E, 0x7D},
	{0x5F, 0x7D},
	{0x60, 0x55},
	{0x61, 0x70},
	{0x62, 0x80},
	{0x7C, 0x05},
	{0x20, 0x80},
	{0x28, 0x30},
	{0x6C, 0x00},
	{0x6D, 0x80},
	{0x6E, 0x00},
	{0x70, 0x02},
	{0x71, 0x94},
	{0x73, 0xC1},
	{0x3D, 0x34},
	{0x5A, 0x57},
	{0x4F, 0xBB},
	{0x50, 0x9C},

	{0xFF, 0x00},
	{0xE5, 0x7F},
	{0xF9, 0xC0},
	{0x41, 0x24},
	{0xE0, 0x14},
	{0x76, 0xFF},
	{0x33, 0xA0},
	{0x42, 0x20},
	{0x43, 0x18},
	{0x4C, 0x00},
	{0x87, 0xD0},
	{0x88, 0x3F},
	{0xD7, 0x03},
	{0xD9, 0x10},
	{0xD3, 0x82},
	{0xC8, 0x08},
	{0xC9, 0x80},
	{0x7C, 0x00},
	{0x7D, 0x00},
	{0x7C, 0x03},
	{0x7D, 0x48},
	{0x7D, 0x48},
	{0x7C, 0x08},
	{0x7D, 0x20},
	{0x7D, 0x10},
	{0x7D, 0x0E},
	{0x90, 0x00},
	{0x91, 0x0E},
	{0x91, 0x1A},
	{0x91, 0x31},
	{0x91, 0x5A},
	{0x91, 0x69},
	{0x91, 0x75},
	{0x91, 0x7E},
	{0x91, 0x88},
	{0x91, 0x8F},
	{0x91, 0x96},
	{0x91, 0xA3},
	{0x91, 0xAF},
	{0x91, 0xC4},
	{0x91, 0xD7},
	{0x91, 0xE8},
	{0x91, 0x20},
	{0x92, 0x00},
	{0x93, 0x06},
	{0x93, 0xE3},
	{0x93, 0x03},
	{0x93, 0x03},
	{0x93, 0x00},
	{0x93, 0x02},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x93, 0x00},
	{0x96, 0x00},
	{0x97, 0x08},
	{0x97, 0x19},
	{0x97, 0x02},
	{0x97, 0x0C},
	{0x97, 0x24},
	{0x97, 0x30},
	{0x97, 0x28},
	{0x97, 0x26},
	{0x97, 0x02},
	{0x97, 0x98},
	{0x97, 0x80},
	{0x97, 0x00},
	{0x97, 0x00},
	{0xA4, 0x00},
	{0xA8, 0x00},
	{0xC5, 0x11},
	{0xC6, 0x51},
	{0xBF, 0x80},
	{0xC7, 0x10},
	{0xB6, 0x66},
	{0xB8, 0xA5},
	{0xB7, 0x64},
	{0xB9, 0x7C},
	{0xB3, 0xAF},
	{0xB4, 0x97},
	{0xB5, 0xFF},
	{0xB0, 0xC5},
	{0xB1, 0x94},
	{0xB2, 0x0F},
	{0xC4, 0x5C},
	{0xA6, 0x00},
	{0xA7, 0x20},
	{0xA7, 0xD8},
	{0xA7, 0x1B},
	{0xA7, 0x31},
	{0xA7, 0x00},
	{0xA7, 0x18},
	{0xA7, 0x20},
	{0xA7, 0xD8},
	{0xA7, 0x19},
	{0xA7, 0x31},
	{0xA7, 0x00},
	{0xA7, 0x18},
	{0xA7, 0x20},
	{0xA7, 0xD8},
	{0xA7, 0x19},
	{0xA7, 0x31},
	{0xA7, 0x00},
	{0xA7, 0x18},
	{0x7F, 0x00},
	{0xE5, 0x1F},
	{0xE1, 0x77},
	{0xDD, 0x7F},
	{0xC2, 0x0E},

	{0x00, 0x00}
};

static const uint8_t svga_regs[][2] = {
	{0xFF, 0x01},
	{0x12, 0x40},
	{0x03, 0x0F},
	{0x32, 0x09},
	{0x17, 0x11},
	{0x18, 0x43},
	{0x19, 0x00},
	{0x1A, 0x4B},
	{0x3D, 0x38},
	{0x35, 0xDA},
	{0x22, 0x1A},
	{0x37, 0xC3},
	{0x34, 0xC0},
	{0x06, 0x88},
	{0x0D, 0x87},
	{0x0E, 0x41},
	{0x42, 0x03},

	{0xFF, 0x00},
	{0x05, 0x01},
	{0xE0, 0x04},
	{0xC0, 0x64},
	{0xC1, 0x4B},
	{0x8C, 0x00},
	{0x53, 0x00},
	{0x54, 0x00},
	{0x51, 0xC8},
	{0x52, 0x96},
	{0x55, 0x00},
	{0x57, 0x00},
	{0x86, 0x3D},
	{0x50, 0x80},
	{0xD3, 0x80},
	{0x05, 0x00},
	{0xE0, 0x00},

	{0x00, 0x00}
};

static const uint8_t jpeg_regs[][2] = {
	{0xFF, 0x00},
	{0xE0, 0x04},
	{0xDA, 0x18},
	{0xD7, 0x03},
	{0xE1, 0x77},
	{0x44, 0x0C},
	{0xE0, 0x00},

	{0x00, 0x00}
};

static const uint8_t framesize_low_regs[][2] = {
	{0xFF, 0x00},
	{0x05, 0x01},
	{0x5A, 0xA0},
	{0x5B, 0x78},
	{0x5C, 0x00},

	{0xFF, 0x01},
	{0x11, 0x83},

	{0x00, 0x00}
};

static const uint8_t framesize_high_regs[][2] = {
	{0xFF, 0x00},
	{0x05, 0x00},

	{0x00, 0x00}
};

static const uint8_t quality_regs[][2] = {
	{0xFF, 0x00},
	{0x44, 0x0F},

	{0x00, 0x00}
};

esp_err_t read_camera_config(uint8_t reg, uint8_t* data){

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, CAMERA_SLAVE_ADDRESS_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t err = i2c_master_cmd_begin(I2C_CAMERA_NUM, cmd, 10 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if(err != ESP_OK){
		return err;
	}

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, CAMERA_SLAVE_ADDRESS_READ, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, data, NACK_VAL);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_CAMERA_NUM, cmd, 10 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if(err != ESP_OK){
		return err;
	}

    return ESP_OK;
}

esp_err_t write_camera_config(uint8_t reg, uint8_t data){

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, CAMERA_SLAVE_ADDRESS_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t err = i2c_master_cmd_begin(I2C_CAMERA_NUM, cmd, 10 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if(err != ESP_OK){
		return err;
	}

    return ESP_OK;
}

static esp_err_t write_frame(http_context_t http_ctx){

    http_buffer_t fb_data = {
		.data = camera_get_fb(),
		.size = camera_get_data_size(),
		.data_is_persistent = true
    };
    return http_response_write(http_ctx, &fb_data);
}

static void handle_jpg(http_context_t http_ctx, void* ctx){

    esp_err_t err = camera_run();
    if(err != ESP_OK){
        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
        return;
    }

    http_response_begin(http_ctx, 200, "image/jpeg", camera_get_data_size());
    http_response_set_header(http_ctx, "Content-disposition", "inline; filename=capture.jpg");
    write_frame(http_ctx);
    http_response_end(http_ctx);
}

static esp_err_t event_handler(void *ctx, system_event_t *event){

    switch(event->event_id){
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
            s_ip_addr = event->event_info.got_ip.ip_info.ip;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void initialise_wifi(void){

    tcpip_adapter_init();
    s_wifi_event_group = xEventGroupCreate();
    esp_event_loop_init(event_handler, NULL);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "Connecting to \"%s\"", wifi_config.sta.ssid);
    xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected");
}

esp_err_t init_camera_clock(camera_config_t* config){

	periph_module_enable(PERIPH_LEDC_MODULE);

	ledc_timer_config_t timer_conf = {
        .duty_resolution = 1,
        .freq_hz = config->xclk_freq_hz,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = config->ledc_timer
	};
	esp_err_t err = ledc_timer_config(&timer_conf);
	if(err != ESP_OK){
		ESP_LOGE(TAG, "ledc_timer_config failed, err=%d", err);
		return err;
	}

	ledc_channel_config_t ch_conf = {
        .channel = config->ledc_channel,
        .timer_sel = config->ledc_timer,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 1,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .gpio_num = config->pin_xclk
	};
	err = ledc_channel_config(&ch_conf);
	if(err != ESP_OK){
		ESP_LOGE(TAG, "ledc_channel_config failed, err=%d", err);
		return err;
	}

	return ESP_OK;
}

void init_i2c(camera_config_t* config){
	int i2c_master_port = I2C_NUM_0;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = config->pin_sscb_sda;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = config->pin_sscb_scl;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_CAMERA_FREQ_HZ;
	i2c_param_config(i2c_master_port, &conf);
	i2c_driver_install(i2c_master_port, conf.mode, I2C_CAMERA_RX_BUF_DISABLE, I2C_CAMERA_TX_BUF_DISABLE, 0);
}

void app_main(){

	esp_log_level_set("*", ESP_LOG_VERBOSE);

    esp_err_t err = nvs_flash_init();
    if(err != ESP_OK){
        nvs_flash_erase();
        nvs_flash_init();
    }

    gpio_install_isr_service(0);

    camera_config_t camera_config = {
		.ledc_channel = LEDC_CHANNEL_0,
		.ledc_timer = LEDC_TIMER_0,
		.pin_d0 = PIN_D0,
		.pin_d1 = PIN_D1,
		.pin_d2 = PIN_D2,
		.pin_d3 = PIN_D3,
		.pin_d4 = PIN_D4,
		.pin_d5 = PIN_D5,
		.pin_d6 = PIN_D6,
		.pin_d7 = PIN_D7,
		.pin_xclk = PIN_XCLK,
		.pin_pclk = PIN_PCLK,
		.pin_vsync = PIN_VSYNC,
		.pin_href = PIN_HREF,
		.pin_sscb_sda = PIN_SDA,
		.pin_sscb_scl = PIN_SCL,
		.pin_reset = PIN_RESET,

		.xclk_freq_hz = 6000000,
		.width = 640,
		.height = 480,
    };

	ESP_LOGE(TAG, "xclk_freq_hz=%d[Hz]", camera_config.xclk_freq_hz);

	//GPIO設定
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	//CAMERA クロック出力
	init_camera_clock(&camera_config);

	//I2C 初期化
	init_i2c(&camera_config);

	//CAMERA リセット
	gpio_set_level(PIN_RESET, 0);
	vTaskDelay(10 / portTICK_RATE_MS);
	gpio_set_level(PIN_RESET, 1);
	vTaskDelay(10 / portTICK_RATE_MS);

	//CAMERA OV2640確認
	uint8_t camera_pid_h;
	read_camera_config(0x0A, &camera_pid_h);
	ESP_LOGI(TAG, "camera_pid_h=0x%02X", camera_pid_h);
    if(camera_pid_h != 0x26){
        ESP_LOGE(TAG, "failed, camera_pid_h=%02X", camera_pid_h);
        return;
    }

	//CAMERA システムリセット
	write_camera_config(0xFF, 0x01);
	write_camera_config(0x12, 0x80);
	vTaskDelay(10 / portTICK_RATE_MS);

	//CAMERA 設定書き込み
    int i = 0;
    while(default_regs[i][0]){
		write_camera_config(default_regs[i][0], default_regs[i][1]);
        i++;
    }
	i = 0;
    while(jpeg_regs[i][0]){
		write_camera_config(jpeg_regs[i][0], jpeg_regs[i][1]);
        i++;
    }
	i = 0;
    while(framesize_low_regs[i][0]){
		write_camera_config(framesize_low_regs[i][0], framesize_low_regs[i][1]);
        i++;
    }
	i = 0;
    while(svga_regs[i][0]){
		write_camera_config(svga_regs[i][0], svga_regs[i][1]);
        i++;
    }
	i = 0;
    while(framesize_high_regs[i][0]){
		write_camera_config(framesize_high_regs[i][0], framesize_high_regs[i][1]);
        i++;
    }
	i = 0;
    while(quality_regs[i][0]){
		write_camera_config(quality_regs[i][0], quality_regs[i][1]);
        i++;
    }

	//CAMERA その他設定
    err = camera_init(&camera_config);
    if(err != ESP_OK){
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

	//CAMERA wifi設定
    initialise_wifi();
    http_server_t server;
    http_server_options_t http_options = HTTP_SERVER_OPTIONS_DEFAULT();
    http_server_start(&http_options, &server);

	http_register_handler(server, "/jpg", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_jpg, NULL);
	ESP_LOGI(TAG, "Open http://" IPSTR "/jpg for single image/jpg image", IP2STR(&s_ip_addr));

    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Camera demo ready");
}