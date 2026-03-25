/**
 * File: Hecate.c
 * 
 * Author(s): Akash Keshav
 * 
 * Purpose: Handle the NFC connections within the Bits 'n Bytes machine.
 * Controls the PN532 reader and reports to the main Raspberry Pi when
 * a valid ID is recieved. Also handles the LED lighting for the cabinet
 * mirror, with a variety of color settings. This module acts as the
 * gatekeeper for entering the cabinet, hence the name Hecate, who guards
 * the gates between the overworld and underworld.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/ledc.h"

#include "sdkconfig.h"
#include "pn532_driver_i2c.h"
#include "pn532.h"

// NFC Definitions
#define SCL_PIN             (22)
#define SDA_PIN             (21)
#define RESET_PIN           (-1)
#define IRQ_PIN             (-1)

#define NFC_TIMEOUT         (0) // 0 is block until ID is found
#define MAX_UID_LEN         (7)
#define FORCE_RESET_PIN     (27) // TODO: Implement Restart via GPIO

// Pi Communication Definitions
#define PI_UART_TX_PIN      (17)
#define PI_UART_RX_PIN      (16)
#define PI_UART_PORT_NUM    UART_NUM_2
#define PI_UART_BAUD_RATE   9600

#define PAYLOAD_SIZE        (7)

// LED Definitions
#define RED_PIN             (12)
#define GREEN_PIN           (13)
#define BLUE_PIN            (14)

#define LED_FREQ            (5000)
#define LED_TIMER           (0)
#define LED_DUTY            (0)
#define LED_HPOINT          (0)
#define RED_CHANNEL         (0)
#define GREEN_CHANNEL       (1)
#define BLUE_CHANNEL        (2)

// Define log tags
static const char *TAG = "BNB_MAIN";
static const char *UART_TAG = "BNB_UART";
static const char *I2C_TAG = "BNB_I2C";
static const char *LED_Tag = "LED";

struct nfc_resp {
    uint8_t uid[MAX_UID_LEN];
    uint8_t length;
};

pn532_io_t pn532_io; 

// minimum size is 128
static const int PI_UART_RX_BUFFER_SIZE = 256; 
static const int PI_UART_TX_BUFFER_SIZE = 256; 

void init_uart() {
    ESP_LOGI(UART_TAG, "Initializing UART for %d-byte binary communication.", PAYLOAD_SIZE);

    ESP_ERROR_CHECK(uart_driver_install(PI_UART_PORT_NUM, PI_UART_RX_BUFFER_SIZE, PI_UART_TX_BUFFER_SIZE, 0, NULL, 0));

    uart_config_t pi_uart_cfg = {
        .baud_rate = PI_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(PI_UART_PORT_NUM, &pi_uart_cfg));

    ESP_ERROR_CHECK(uart_set_pin(PI_UART_PORT_NUM, PI_UART_TX_PIN, PI_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(UART_TAG, "UART initialized. Sending %d-byte payloads.", PAYLOAD_SIZE);
}

int uart_send_data(const uint8_t* data) {
    return uart_write_bytes(PI_UART_PORT_NUM, (const char*)data, PAYLOAD_SIZE);
}

void pn532_i2c_init() {
    esp_err_t err;
    
    ESP_LOGI(I2C_TAG, "Initializing PN532...");
    ESP_ERROR_CHECK(pn532_new_driver_i2c(SDA_PIN, SCL_PIN, RESET_PIN, IRQ_PIN, 0, &pn532_io));

    do {
        err = pn532_init(&pn532_io);
        if (err != ESP_OK) {
            ESP_LOGW(I2C_TAG, "Failed to initialize PN532. Retrying...");
            pn532_release(&pn532_io);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while (err != ESP_OK);

    uint32_t version_data = 0;
    ESP_ERROR_CHECK(pn532_get_firmware_version(&pn532_io, &version_data));
    ESP_LOGI(I2C_TAG, "PN53x Found (Ver. %d.%d)", (int)(version_data >> 16) & 0xFF, (int)(version_data >> 8) & 0xFF);
}

struct nfc_resp run_nfc() {
    struct nfc_resp r = {{0}, 0}; 
    esp_err_t err;

    ESP_LOGI(I2C_TAG, "Waiting for ISO14443A Card...");

    err = pn532_read_passive_target_id(&pn532_io, PN532_BRTY_ISO14443A_106KBPS, r.uid, &r.length, NFC_TIMEOUT);

    if (ESP_OK == err) {
        ESP_LOGI(I2C_TAG, "UID Length: %d", r.length);
        ESP_LOG_BUFFER_HEX_LEVEL(I2C_TAG, r.uid, r.length, ESP_LOG_INFO);
    } else {
        ESP_LOGE(I2C_TAG, "Error reading UID: %d: %s", err, esp_err_to_name(err));
        memset(r.uid,0xFF,MAX_UID_LEN);
        r.length = 0;
    }

    return r;
}

static void IRAM_ATTR on_reset_gpio(void* arg) {
    ESP_EARLY_LOGI("ISR", "RESTART command recieved!", (int)arg);
    // esp_restart();
}

void init_gpio() {
    esp_err_t err;
    gpio_config_t reset_conf = {
        .pin_bit_mask = (1UL << FORCE_RESET_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };

    gpio_install_isr_service(0);
    gpio_isr_handler_add(FORCE_RESET_PIN, on_reset_gpio, (void*) FORCE_RESET_PIN);

    ESP_ERROR_CHECK(gpio_config(&reset_conf));

    ESP_LOGI(UART_TAG, "GPIO Reset Initialized!");
}

void init_led() {
    // create config and apply it
    ledc_timer_config_t timer_config = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LED_TIMER,
        .freq_hz          = LED_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);

    // red channel config
    ledc_channel_config_t cnl_conf_red = {
        .gpio_num       =   RED_PIN,
        .speed_mode     =   LEDC_HIGH_SPEED_MODE,
        .channel        =   RED_CHANNEL,
        .timer_sel      =   LED_TIMER,
        .duty           =   0,
        .hpoint         =   0
    };
    // green channel config
    ledc_channel_config_t cnl_conf_green = {
        .gpio_num       =   GREEN_PIN,
        .speed_mode     =   LEDC_HIGH_SPEED_MODE,
        .channel        =   GREEN_CHANNEL,
        .timer_sel      =   LED_TIMER,
        .duty           =   0,
        .hpoint         =   0
    };
    // blue channel config
    ledc_channel_config_t cnl_conf_blue = {
        .gpio_num       =   BLUE_PIN,
        .speed_mode     =   LEDC_HIGH_SPEED_MODE,
        .channel        =   BLUE_CHANNEL,
        .timer_sel      =   LEDC_TIMER_0,
        .duty           =   0,
        .hpoint         =   0
    };

    // apply rgb configs
    ledc_channel_config(&cnl_conf_red);
    ledc_channel_config(&cnl_conf_green);
    ledc_channel_config(&cnl_conf_blue);
}

void setColor(uint8_t r, uint8_t g, uint8_t b) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, r);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, g);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, b);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
}

void app_main() {
    struct nfc_resp response;
    esp_err_t err;

    memset(&response, 0, sizeof(struct nfc_resp));

    pn532_i2c_init();
    init_uart();
    init_gpio();

    bool color_toggle = false; // false = orange, true = purple
    
    while (1) {
        uint8_t *tx_payload = (uint8_t *) malloc(PI_UART_RX_BUFFER_SIZE);
        
        int rx_len = uart_read_bytes(PI_UART_PORT_NUM, tx_payload, PAYLOAD_SIZE, 50 / portTICK_PERIOD_MS);

        ESP_LOGI(UART_TAG, "ACK Received from Pi (RX)! %d", rx_len);
        if (rx_len > 0) {
            ESP_LOG_BUFFER_HEX(UART_TAG, tx_payload, sizeof(tx_payload));
        }
        
        // correct length response and first byte is correct
        if (rx_len == PAYLOAD_SIZE && tx_payload[0] == 0xFF) {
            ESP_LOGI(UART_TAG, "ACK Received from Pi (RX): Valid 8-byte packet.");
            ESP_LOG_BUFFER_HEXDUMP(UART_TAG, tx_payload, sizeof(tx_payload), ESP_LOG_INFO);

            response = run_nfc();

            uart_send_data(response.uid);
            ESP_LOG_BUFFER_HEXDUMP(UART_TAG, response.uid, response.length, ESP_LOG_DEBUG);
        }

        // Swap between the colors
        if (color_toggle) {
            setColor(128,0,128);
        } else {
            setColor(255,165,0);
        }
        color_toggle = !color_toggle;

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
}