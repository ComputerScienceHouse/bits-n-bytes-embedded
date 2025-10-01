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
// TODO discover shelves over ESP-NOW
// TODO send/receive UART packets from raspberry pi

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/uart.h"
#include <string.h>
#include "nvs_flash.h"
#include "led_strip.h"
#include <math.h>
#include "esp_timer.h"
#include "cJSON.h"

// Pins
#define INTERNAL_LED_PIN 2
#define CABINET_LED_PIN 16
#define DOOR_CONTROL_PIN 5
#define LEFT_DOOR_UPPER_SENSOR_PIN 23
#define LEFT_DOOR_LOWER_SENSOR_PIN 19
#define RIGHT_DOOR_UPPER_SENSOR_PIN 18
#define RIGHT_DOOR_LOWER_SENSOR_PIN 26
#define HATCH_CONTROL_PIN 32
#define HATCH_LEFT_SENSOR_PIN 12
#define HATCH_RIGHT_SENSOR_PIN 4
#define UART_TX_PIN 9
#define UART_RX_PIN 10

// UART
#define PI_UART_PORT_NUM 2
#define PI_UART_BAUD_RATE 115200
static const int pi_uart_rx_buffer_size = 1024;

// Timing
#define DOOR_TRIGGER_DELAY_MS 100 // Leaving latches open for too long is harmful
#define HATCH_TRIGGER_DELAY_MS 100

// LEDs
#define NUM_LEDS 128 // Number of LEDs on each side of the cabinet (individual strips)

// Tag for log messages
static const char* TAG = "atlas";

// Handle for cabinet LED strip
led_strip_handle_t led_strip = NULL;


// ESP-NOW
static QueueHandle_t esp_now_q;
typedef struct {
    uint8_t data[250];
    int len;
} esp_now_msg_t;


/**
 * Open the doors
 */
void open_doors() {
    // Set the open doors last send to prevent this from triggering the doors open
    gpio_set_level(INTERNAL_LED_PIN, 0);
    gpio_set_level(DOOR_CONTROL_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(DOOR_TRIGGER_DELAY_MS));
    gpio_set_level(DOOR_CONTROL_PIN, 1);
    gpio_set_level(INTERNAL_LED_PIN, 1);
}

/**
 * Open the hatch
 */
void open_hatch() {
    gpio_set_level(INTERNAL_LED_PIN, 0);
    gpio_set_level(HATCH_CONTROL_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(HATCH_TRIGGER_DELAY_MS));
    gpio_set_level(HATCH_CONTROL_PIN, 1);
    gpio_set_level(INTERNAL_LED_PIN, 1);
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



void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!data || len <= 0) return;

    esp_now_msg_t msg;
    if (len > sizeof(msg.data)) len = sizeof(msg.data);
    memcpy(msg.data, data, len);
    msg.len = len;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(esp_now_q, &msg, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void esp_now_task(void *pvParameter) {
    esp_now_msg_t msg;
    while (true) {
        if (xQueueReceive(esp_now_q, &msg, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Received %d bytes over ESP-NOW", msg.len);
            for (size_t i = 0; i < msg.len; i++) {
                ESP_LOGI(TAG, "%d", msg.data[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
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
            .pin_bit_mask = (1ULL << INTERNAL_LED_PIN),
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
    esp_netif_init();
    esp_event_loop_create_default();
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
    ESP_ERROR_CHECK(uart_driver_install(PI_UART_PORT_NUM, pi_uart_rx_buffer_size, 0, 0, NULL, 0));
    const uart_port_t uart_num = PI_UART_PORT_NUM;
    uart_config_t uart_cfg = {
            .baud_rate = PI_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(PI_UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


/**
 * Initialize the LEDs.
 */
void init_leds() {

    led_strip_config_t strip_config = {
            .strip_gpio_num = CABINET_LED_PIN,
            .max_leds = NUM_LEDS,
            .led_model = LED_MODEL_WS2812,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .flags = {
                    .invert_out = false
            }
    };
    led_strip_rmt_config_t rmt_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 10 * 1000 * 1000,
            .flags.with_dma = false,
    };
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
}


/**
 * Convert HSV to RGB.
 * @param h float hue
 * @param s float saturation
 * @param v float value
 * @param r pointer to uint8_t for red
 * @param g pointer to uint8_t for green
 * @param b pointer to uint8_t for blue
 */
static void hsv_to_rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    float c = v * s;
    float x = c * (1 - fabsf(fmodf(h / 60.0f, 2) - 1));
    float m = v - c;

    float r1, g1, b1;
    if (h < 60)      { r1 = c; g1 = x; b1 = 0; }
    else if (h < 120){ r1 = x; g1 = c; b1 = 0; }
    else if (h < 180){ r1 = 0; g1 = c; b1 = x; }
    else if (h < 240){ r1 = 0; g1 = x; b1 = c; }
    else if (h < 300){ r1 = x; g1 = 0; b1 = c; }
    else             { r1 = c; g1 = 0; b1 = x; }

    *r = (uint8_t)((r1 + m) * 255);
    *g = (uint8_t)((g1 + m) * 255);
    *b = (uint8_t)((b1 + m) * 255);
}


/**
 * Rainbow effect for the LEDs. This function must be called in a loop, and the
 * the hue_offset value must be stored and passed in each time this is called
 * in order for the LEDs to fade properly.
 * @param hue_offset Hue offset (result from last call of this function, use
 * 0.0f if this is the first time this function is called).
 * @return hue_offset to be stored and then passed into the next iteration.
 */
float leds_rainbow_chase(float hue_offset) {
    esp_err_t err;
    // Fast effect
    for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t r, g, b;
        float hue = fmodf(hue_offset + (360.0f * i / NUM_LEDS), 360.0f);
        hsv_to_rgb(hue, 1.0f, 1.0f, &r, &g, &b);
        err = led_strip_set_pixel(led_strip, i, r, g, b);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Unable to set pixel %d of LED", i);
        }
    }
    err = led_strip_refresh(led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error refreshing LEDs in leds_rainbow_chase");
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    // Update hue offset
    hue_offset = fmodf(hue_offset + 2.0f, 360.0f);
    return hue_offset;
}


/**
 * Color chase of the official Bits 'n Bytes colors
 * @param phase float phase to carry over from the last iteration
 * @return float phase to carry over to the next iteration
 */
float leds_bnb_color_chase(float phase) {
    esp_err_t err;
    const int BLOCK_SIZE = 10;

    int offset = ((int)phase) % (BLOCK_SIZE * 2); // wrap around

    for (int i = 0; i < NUM_LEDS; i++) {
        // Determine which block this LED is in
        int block_index = (i + offset) / BLOCK_SIZE;

        if (block_index % 2 == 0) {
            // Orange block
            err = led_strip_set_pixel(led_strip, i, 250, 25, 0);
        } else {
            // Purple block
            err = led_strip_set_pixel(led_strip, i, 130, 0, 250);
        }

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Unable to set pixel %d of LED", i);
        }
    }
    err = led_strip_refresh(led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error refreshing LEDs in leds_red");
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    return phase + 1;
}


float leds_rainbow_gradient(float hue) {
    esp_err_t err;
    uint8_t r, g, b;
    hue = fmodf(hue + 2.0f, 360.0f);
    for (int i = 0; i < NUM_LEDS; i++) {
        hsv_to_rgb(hue, 1.0f, 1.0f, &r, &g, &b);
        led_strip_set_pixel(led_strip, i, r, g, b);
    }
    err = led_strip_refresh(led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error refreshing LEDs in leds_red");
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    return hue;
}



/**
 * Task to update LEDs based on state of cabinet.
 * @return This is a task and it will never return.
 */
_Noreturn void update_leds_task() {

    // Array of all effects to cycle through
    float (*effects[])(float) = {leds_rainbow_chase, leds_bnb_color_chase, leds_rainbow_gradient};
    // An effect might need to store a value for it to reference each
    // time the effect is called (e.g. current hue for rainbow effect)
    float fx_val_store = 0.0f;

    uint16_t current_effect = 0;

    int effect_switch_ms = 7000;
    int64_t effect_last_changed = 0;

    while(1) {
        if (are_doors_closed()) {
            // Attract mode: Do cool effects

            // Check if it's time to switch effects
            int64_t current_time_ms = esp_timer_get_time() / 1000;
            if (current_time_ms - effect_last_changed > effect_switch_ms) {
                // Switch effects
                current_effect = (current_effect + 1) % (sizeof(effects) / sizeof(effects[0]));
                effect_last_changed = current_time_ms;
                fx_val_store = 0.0f;
            }

            // Update LEDs based on current effect
            fx_val_store = effects[current_effect](fx_val_store);

        } else {
            for (size_t i = 0; i < NUM_LEDS; i++) {
                led_strip_set_pixel(led_strip, i, 255, 255, 255);
            }
            led_strip_refresh(led_strip);
            // Task can be delayed longer, LED updates are not important
            // (they stay the same while the doors are open).
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/**
 * One-shot task to open the doors.
 */
void open_doors_task() {
    open_doors();
    vTaskDelete(NULL);
}

/**
 * One-shot task to open the hatch.
 */
void open_hatch_task() {
    open_hatch();
    vTaskDelete(NULL);
}


/**
 * Task to read data from uart and write status.
 * @return This is a task and it will never return.
 */
_Noreturn void controller_read_write_uart_status_task() {

    const char* tag = "controller_uart";


    while (1) {

        esp_err_t err;

        // Read packets from controller
        char data[pi_uart_rx_buffer_size];
        size_t length = 0;
        err = uart_get_buffered_data_len(PI_UART_PORT_NUM, &length);
        if (err != ESP_OK) {
            // Skip to sending status
            ESP_LOGW(tag, "Unable to get buffered data length");
            goto send_statuses;
        }
        if (length > 0) {
            uart_read_bytes(PI_UART_PORT_NUM, data, length, pdMS_TO_TICKS(10));

            // Parse JSON
            cJSON *json = cJSON_ParseWithLength(data, length);
            if (json == NULL) {
                ESP_LOGW(tag, "Unable to parse JSON");
                goto send_statuses;
            }
            const cJSON *doors_item = NULL;
            const cJSON *hatch_item = NULL;
            doors_item = cJSON_GetObjectItem(json, "doors");
            hatch_item = cJSON_GetObjectItem(json, "hatch");


            // Check if doors need to be opened
            if (!cJSON_IsBool(doors_item)) {
                ESP_LOGW(tag, "JSON value for 'doors' is not a boolean.");
            } else {
                if (doors_item->valueint == 1) {
                    // Schedule call to open doors
                    xTaskCreate(open_doors_task, "open_doors_task", 1028, NULL, 5, NULL);
                }
            }

            // Check if hatch needs to be opened
            if (!cJSON_IsBool(hatch_item)) {
                ESP_LOGW(tag, "JSON value for 'hatch' is not a boolean.");
            } else {
                if (hatch_item->valueint == 1) {
                    // Schedule call to open hatch
                    xTaskCreate(open_hatch_task, "open_hatch_task", 1028, NULL, 5, NULL);
                }
            }
        }


        send_statuses:
        // TODO write statuses
        vTaskDelay(pdMS_TO_TICKS(20));
    }
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

    nvs_flash_init();

    configure_pins();
    ESP_LOGD(TAG, "Initialized GPIO");

    init_wifi();
    ESP_LOGD(TAG, "Initialized wifi radio");

    init_uart();
    ESP_LOGD(TAG, "Initialized UART");

    init_leds();
    ESP_LOGD(TAG, "Initialized LEDs");

    // Schedule LEDs control task
    xTaskCreate(update_leds_task, "update_leds_task", 4096, NULL, 4, NULL);


    // Schedule UART read/write tasks
    xTaskCreate(controller_read_write_uart_status_task, "controller_read_write_task", 4096, NULL, 5, NULL);



    // ESP-NOW

    esp_now_q = xQueueCreate(10, sizeof(esp_now_msg_t));
//    esp_now_register_recv_cb(esp_now_recv_cb);
//    xTaskCreate(esp_now_task, "esp_now_task", 4096, NULL, 5, NULL);


}
