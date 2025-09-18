/**
 * File: atlas.c
 *
 * Author(s): Isaac Ingram
 *
 * Purpose: Control ALL of the embedded aspects of the Bits 'n Bytes cabinet.
 * This includes the doors, the hatch, the LEDs, the fans, and aggregating
 * weight data from all shelves via ESP-NOW. This ESP does a lot of heavy
 * lifting, thus it is named Atlas.
 */

// TODO add noctua fan contrOl
// TODO add LED control
// TODO discover shelves over ESP-NOW
// TODO send/receive UART packets from raspberry pi

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/uart.h"

// Pins
#define LED_PIN 2
#define DOOR_CONTROL_PIN 5
#define LEFT_DOOR_UPPER_SENSOR_PIN 11
#define LEFT_DOOR_LOWER_SENSOR_PIN 1
#define RIGHT_DOOR_UPPER_SENSOR_PIN 9
#define RIGHT_DOOR_LOWER_SENSOR_PIN 2
#define HATCH_CONTROL_PIN 7
#define HATCH_LEFT_SENSOR_PIN 3
#define HATCH_RIGHT_SENSOR_PIN 4
#define UART_TX_PIN 1
#define UART_RX_PIN 2

// UART
#define UART_PORT_NUM 1
#define UART_BAUD_RATE 115200
static const int uart_rx_buffer_size = 1024;

// Timing
#define DOOR_TRIGGER_DELAY_MS 100 // Leaving latches open for too long is harmful
#define HATCH_TRIGGER_DELAY_MS 100


static const char* TAG = "atlas";

/**
 * Open the doors
 */
void open_doors() {
    // Set the open doors last send to prevent this from triggering the doors open
    gpio_set_level(LED_PIN, 0);
    gpio_set_level(DOOR_CONTROL_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(DOOR_TRIGGER_DELAY_MS));
    gpio_set_level(DOOR_CONTROL_PIN, 1);
    gpio_set_level(LED_PIN, 1);
}

/**
 * Open the hatch
 */
void open_hatch() {
    gpio_set_level(LED_PIN, 0);
    gpio_set_level(HATCH_CONTROL_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(HATCH_TRIGGER_DELAY_MS));
    gpio_set_level(HATCH_CONTROL_PIN, 1);
    gpio_set_level(LED_PIN, 1);
}

/**
 * Check if the left door is closed
 * @return boolean
 */
bool is_left_door_closed() {
    return !gpio_get_level(LEFT_DOOR_UPPER_SENSOR_PIN) && !gpio_get_level(LEFT_DOOR_LOWER_SENSOR_PIN);
}

/**
 * Check if the right door is closed
 * @return boolean
 */
bool is_right_door_closed() {
    return !gpio_get_level(RIGHT_DOOR_UPPER_SENSOR_PIN) && !gpio_get_level(RIGHT_DOOR_LOWER_SENSOR_PIN);
}

/**
 * Check if both doors are closed
 *
 * @return boolean
 */
bool are_doors_closed() {
    return is_left_door_closed() && is_right_door_closed();
}

/**
 * Check if the left hatch is closed
 * @return boolean
 */
bool is_left_hatch_closed() {
    return !gpio_get_level(HATCH_LEFT_SENSOR_PIN);
}

/**
 * Check if the right hatch is closed
 * @return boolean
 */
bool is_right_hatch_closed() {
    return !gpio_get_level(HATCH_RIGHT_SENSOR_PIN);
}

/**
 * Check if the hatch is closed
 * @return boolean
 */
bool is_hatch_closed() {
    return is_left_hatch_closed() && is_right_hatch_closed();
}


/**
 * Configure GPIO pins.
 */
void configure_pins() {
    // Configure the latch control pins.
    gpio_config_t control_pin_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << DOOR_CONTROL_PIN) |
                            (1ULL << HATCH_CONTROL_PIN),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&control_pin_conf);

    // Configure the latch sensor pins.
    gpio_config_t sensor_pin_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << LEFT_DOOR_UPPER_SENSOR_PIN) |
                            (1ULL << LEFT_DOOR_LOWER_SENSOR_PIN) |
                            (1ULL << RIGHT_DOOR_UPPER_SENSOR_PIN) |
                            (1ULL << RIGHT_DOOR_LOWER_SENSOR_PIN) |
                            (1ULL << HATCH_LEFT_SENSOR_PIN) |
                            (1ULL << HATCH_RIGHT_SENSOR_PIN),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&sensor_pin_conf);
    gpio_config_t led_pin_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL < LED_PIN,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&led_pin_conf);
}


/**
 * Initialize wifi. Note that the Wifi radio must be turned on by calling this
 * function if you desire to use ESP-NOW, even if you will NOT be connecting
 * this ESP to a wifi network.
 */
void init_wifi () {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Wifi!");
    }
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Wifi mode");
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Wifi");
    }
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ESP-NOW");

    }
}


/**
 * Initialize UART.
 */
void init_uart() {
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, uart_rx_buffer_size, 0, 0, NULL, 0));
    const uart_port_t uart_num = UART_PORT_NUM;
    uart_config_t uart_cfg = {
            .baud_rate = UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
            .rx_flow_ctrl_thresh = 122
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


/**
 * Main app code. Runs once on boot.
 */
void app_main(void)
{

    // Set door pins as soon as this ESP boots. This MUST be done before
    // anything else, because if the GPIO level of these pins is 0, the
    // doors WILL open, even in the very short time between cabinet powering
    // up and this ESP booting.
    gpio_set_level(DOOR_CONTROL_PIN, 1);
    gpio_set_level(HATCH_CONTROL_PIN, 1);

    configure_pins();
    ESP_LOGD(TAG, "Initialized GPIO");

    init_wifi();
    ESP_LOGD(TAG, "Initialized wifi radio");

    init_uart();
    ESP_LOGD(TAG, "Initialized UART");


}
