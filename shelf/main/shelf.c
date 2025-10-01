#include <stdio.h>
#include <mqtt_client.h>
#include <driver/gpio.h>
#include "secrets.h"
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_wifi_default.h>
#include <esp_wifi.h>
#include "hx711.h"
#include <esp_mac.h>
#include "esp_log.h"
#include "esp_now.h"

// Timing definition
#define WEIGHT_UPDATE_DELAY_MS 250
#define LOAD_CELL_READ_TIMEOUT_MS 125
#define LOAD_CELL_SCALING_FACTOR 0.01

// Pin definitions
#define LED_PIN 2
#define LC_CLOCK_PIN 16
#define SLOT_0_UPPER_PIN 35
#define SLOT_0_LOWER_PIN 33
#define SLOT_1_UPPER_PIN 34
#define SLOT_1_LOWER_PIN 14
#define SLOT_2_UPPER_PIN 25
#define SLOT_2_LOWER_PIN 32
#define SLOT_3_UPPER_PIN 27
#define SLOT_3_LOWER_PIN 4


hx711_t slot_0_upper;
hx711_t slot_0_lower;
hx711_t slot_1_upper;
hx711_t slot_1_lower;
hx711_t slot_2_upper;
hx711_t slot_2_lower;
hx711_t slot_3_upper;
hx711_t slot_3_lower;

// TODO remove atlas mac, this should not be hardcoded
uint8_t atlas_mac[ESP_NOW_ETH_ALEN] = {0x30, 0xC6, 0xF7, 0x29, 0xE9, 0xC8};



static char mac_address_str[18];

static const char* TAG = "shelf";


/**
 * Configure GPIO pins
 */
void configure_pins() {
    // Set LED
    gpio_config_t led_pin = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << LED_PIN,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&led_pin);
}


/**
 * Read data from a load cell. Note that if the data is unable to be read, the
 * data will not be modified so the last set value will be used.
 * @param load_cell_t Pointer to hx711_t to read
 * @param output Pointer to int32_t to output value
 * @return esp_err_t
 */
esp_err_t read_load_cell_data(hx711_t* load_cell_t, int32_t* output) {
    esp_err_t err = hx711_wait(load_cell_t, LOAD_CELL_READ_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    err = hx711_read_data(load_cell_t, output);
    return err;
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
 * Store MAC address
 */
void store_mac_address() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(mac_address_str, sizeof(mac_address_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}


/**
 * Initialized the load cells.
 */
void init_load_cells() {
    slot_0_upper.dout = SLOT_0_UPPER_PIN;
    slot_0_upper.pd_sck = LC_CLOCK_PIN;
    slot_0_upper.gain = HX711_GAIN_A_64;
    ESP_ERROR_CHECK(hx711_init(&slot_0_upper));

    slot_0_lower.dout = SLOT_0_LOWER_PIN;
    slot_0_lower.pd_sck = LC_CLOCK_PIN;
    slot_0_lower.gain = HX711_GAIN_A_64;
    ESP_ERROR_CHECK(hx711_init(&slot_0_lower));

    slot_1_upper.dout = SLOT_1_UPPER_PIN,
            slot_1_upper.pd_sck = LC_CLOCK_PIN;
    slot_1_upper.gain = HX711_GAIN_A_64;
    ESP_ERROR_CHECK(hx711_init(&slot_1_upper));

    slot_1_lower.dout = SLOT_1_LOWER_PIN;
    slot_1_lower.pd_sck = LC_CLOCK_PIN;
    slot_1_lower.gain = HX711_GAIN_A_64;
    ESP_ERROR_CHECK(hx711_init(&slot_1_lower));

    slot_2_upper.dout = SLOT_2_UPPER_PIN;
    slot_2_upper.pd_sck = LC_CLOCK_PIN;
    slot_2_upper.gain = HX711_GAIN_A_64;
    ESP_ERROR_CHECK(hx711_init(&slot_2_upper));

    slot_2_lower.dout = SLOT_2_LOWER_PIN,
            slot_2_lower.pd_sck = LC_CLOCK_PIN;
    slot_2_lower.gain = HX711_GAIN_A_64;
    ESP_ERROR_CHECK(hx711_init(&slot_2_lower));

    slot_3_upper.dout = SLOT_3_UPPER_PIN;
    slot_3_upper.pd_sck = LC_CLOCK_PIN;
    slot_3_upper.gain = HX711_GAIN_A_64;
    ESP_ERROR_CHECK(hx711_init(&slot_3_upper));

    slot_3_lower.dout = SLOT_3_LOWER_PIN;
    slot_3_lower.pd_sck = LC_CLOCK_PIN;
    slot_3_lower.gain = HX711_GAIN_A_64;
    ESP_ERROR_CHECK(hx711_init(&slot_3_lower));
}


static void pack_int32_to_int8(const int32_t *src, uint8_t *dst, size_t count) {
    for (size_t i = 0; i < count; i++) {
        int32_t val = src[i];
        dst[i * 4 + 0] = (uint8_t)((val >> 0) & 0xFF);
        dst[i * 4 + 1] = (uint8_t)((val >> 8) & 0xFF);
        dst[i * 4 + 2] = (uint8_t)((val >> 16) & 0xFF);
        dst[i * 4 + 3] = (uint8_t)((val >> 24) & 0xFF);
    }
}


/**
 * Task to get the weights from the load cells and send them over ESP-NOW.
 * @param pvParameters parameters
 * @return This function is a task and it will never return.
 */
_Noreturn void send_weights_task(void* pvParameters) {




    while (1) {

        // Read load cells
        int32_t raw_values[8]; // Array for storing all values

        // Get shelf 0
        read_load_cell_data(&slot_0_upper, &raw_values[0]);
        read_load_cell_data(&slot_0_lower, &raw_values[1]);
        read_load_cell_data(&slot_1_upper, &raw_values[2]);
        read_load_cell_data(&slot_1_lower, &raw_values[3]);
        read_load_cell_data(&slot_2_upper, &raw_values[4]);
        read_load_cell_data(&slot_2_lower, &raw_values[5]);
        read_load_cell_data(&slot_3_upper, &raw_values[6]);
        read_load_cell_data(&slot_3_lower, &raw_values[7]);

        // TODO apply some kind of filtering on the edge

        // Convert data from int32_t to int8_t
        size_t count = sizeof(raw_values) / sizeof(raw_values[0]);
        uint8_t data8[count*4];
        pack_int32_to_int8(raw_values, data8, count);


        esp_now_send(atlas_mac, data8, sizeof(data8));

        gpio_set_level(LED_PIN, 1);

        vTaskDelay(pdMS_TO_TICKS(WEIGHT_UPDATE_DELAY_MS));
    }

}


/**
 * Main function, runs once on app start.
 */
void app_main(void)
{

    nvs_flash_init();
    ESP_LOGD(TAG, "Initialized NVS");

    configure_pins();
    ESP_LOGD(TAG, "Initialized GPIO");

    init_wifi();
    ESP_LOGD(TAG, "Initialized Wifi radio");

    esp_now_init();
    ESP_LOGD(TAG, "Initialized ESP-NOW");

    store_mac_address();

    init_load_cells();
    ESP_LOGD(TAG, "Initialized load cells");


    // Add ESP-NOW peer (Atlas)
    esp_now_peer_info_t en_peer_info = {
            .channel = 0,
            .ifidx = ESP_IF_WIFI_STA,
            .encrypt = false
    };
    memcpy(en_peer_info.peer_addr, atlas_mac, ESP_NOW_ETH_ALEN);
    if (!esp_now_is_peer_exist(atlas_mac)) {
        esp_now_add_peer(&en_peer_info);
    }

    xTaskCreate(send_weights_task, "send_weights_task", 2048, NULL, 4, NULL);

}
