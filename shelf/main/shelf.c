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
#include "cJSON.h"

#define NUM_SLOTS 4

// Timing definition
#define WEIGHT_UPDATE_DELAY_MS 250
#define LOAD_CELL_READ_TIMEOUT_MS 125

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

typedef struct {
    hx711_t *upper_load_cell;
    hx711_t *lower_load_cell;
} slot_t ;

slot_t *slots = NULL;


// TODO remove atlas mac, this should not be hardcoded
uint8_t atlas_mac[ESP_NOW_ETH_ALEN] = {0x30, 0xC6, 0xF7, 0x29, 0xE1, 0x90};



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

    // Allocate memory for slots
    slots = heap_caps_malloc(NUM_SLOTS * sizeof(slot_t), MALLOC_CAP_DEFAULT);
    if (slots == NULL) {
        ESP_LOGE(TAG, "Failed to allocate slots!");
        abort();
    }

    memset(slots, 0, NUM_SLOTS * sizeof(slot_t));

    for (size_t i = 0; i < NUM_SLOTS; i++) {
        // Allocate memory for hx711s
        slots[i].upper_load_cell = heap_caps_malloc(sizeof(hx711_t), MALLOC_CAP_DEFAULT);
        slots[i].lower_load_cell = heap_caps_malloc(sizeof(hx711_t), MALLOC_CAP_DEFAULT);

        if (
                slots[i].upper_load_cell == NULL ||
                slots[i].lower_load_cell == NULL
                ) {
            ESP_LOGE(TAG, "Failed to allocate hx711s!");
            abort();
        }

        // Get hard coded pins
        int upper_pin, lower_pin;
        switch (i) {
            case 0:
                lower_pin = SLOT_0_LOWER_PIN;
                upper_pin = SLOT_0_UPPER_PIN;
                break;
            case 1:
                lower_pin = SLOT_1_LOWER_PIN;
                upper_pin = SLOT_1_UPPER_PIN;
                break;
            case 2:
                lower_pin = SLOT_2_LOWER_PIN;
                upper_pin = SLOT_2_UPPER_PIN;
                break;
            case 3:
                lower_pin = SLOT_3_LOWER_PIN;
                upper_pin = SLOT_3_UPPER_PIN;
                break;
            default:
                ESP_LOGE(TAG, "Unable to initialize load cells in slot %d: Not enough pins defined", i);
                return;
        }

        slots[i].lower_load_cell->dout = lower_pin;
        slots[i].lower_load_cell->pd_sck = LC_CLOCK_PIN;
        slots[i].lower_load_cell->gain = HX711_GAIN_A_64;

        slots[i].upper_load_cell->dout = upper_pin;
        slots[i].upper_load_cell->pd_sck = LC_CLOCK_PIN;
        slots[i].upper_load_cell->gain = HX711_GAIN_A_64;

    }
}

/**
 * Task to get the weights from the load cells and send them over ESP-NOW.
 * @param pvParameters parameters
 * @return This function is a task and it will never return.
 */
_Noreturn void send_weights_task(void* pvParameters) {

    while (1) {

        // Create JSON
        cJSON *json = cJSON_CreateObject();
        cJSON *shelves_array = cJSON_AddArrayToObject(json, "shelves");

        // Read load cell data for each slot
        for (size_t i = 0; i < NUM_SLOTS; i++) {
            int32_t upper, lower;
            if (
                    read_load_cell_data(slots[i].lower_load_cell, &lower) != ESP_OK ||
                    read_load_cell_data(slots[i].upper_load_cell, &upper) != ESP_OK
            )
            {
                // Error reading these load cells, add null value to data
                ESP_LOGW(TAG, "Error reading load cells in slot %d", i);
                cJSON_AddItemToArray(shelves_array, cJSON_CreateNull());
            } else {
                cJSON *weight_value = cJSON_CreateNumber((double)upper + (double)lower);
                cJSON_AddItemToArray(shelves_array, weight_value);
            }
        }

        // Send JSON data to Atlas ESP32
        char *json_string = cJSON_Print(json);
        esp_now_send(atlas_mac, (uint8_t*)json_string, strlen(json_string));

        // Free all memory
        free(json_string);
        cJSON_Delete(json);

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
