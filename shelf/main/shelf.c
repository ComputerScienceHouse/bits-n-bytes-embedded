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
#include "math.h"

// Slots
#define NUM_SLOTS 4

// ESP-NOW buffer
#define ESP_NOW_BUFFER_SIZE 300

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

// Load cells as slots
typedef struct {
    _Atomic double calibration_factor;
    _Atomic double calibration_previous_raw;
    _Atomic double current_raw;
    hx711_t *upper_load_cell;
    hx711_t *lower_load_cell;
} slot_t ;
slot_t *slots = NULL;

// TODO remove atlas mac, this should not be hardcoded
uint8_t atlas_mac[ESP_NOW_ETH_ALEN] = {0x30, 0xC6, 0xF7, 0x29, 0xE1, 0x90};

// String to store this device's mac address
static char mac_address_str[18];

// ESP-NOW receive messages
typedef struct message {
    int32_t length;
    char data[ESP_NOW_BUFFER_SIZE];
} message_t;
static QueueHandle_t esp_now_queue;

// Logging tag
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
    slots = malloc(NUM_SLOTS * sizeof(slot_t));
    if (slots == NULL) {
        ESP_LOGE(TAG, "Failed to allocate slots!");
        abort();
    }

    memset(slots, 0, NUM_SLOTS * sizeof(slot_t));

    for (size_t i = 0; i < NUM_SLOTS; i++) {

        // Allocate memory for hx711s
        slots[i].upper_load_cell = malloc(sizeof(hx711_t));
        slots[i].lower_load_cell = malloc(sizeof(hx711_t));

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
        hx711_init(slots[i].lower_load_cell);

        slots[i].upper_load_cell->dout = upper_pin;
        slots[i].upper_load_cell->pd_sck = LC_CLOCK_PIN;
        slots[i].upper_load_cell->gain = HX711_GAIN_A_64;
        hx711_init(slots[i].upper_load_cell);

        slots[i].calibration_previous_raw = 0;
        slots[i].current_raw = 0;
    }
}

void load_calibration() {

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load calibration values from NVS");
        abort();
    }

    for (size_t i = 0; i < NUM_SLOTS; i++) {

        char storage_key[20];
        sprintf(storage_key, "slot_%d_cal", i);
        double calibration_value = 1.0;
        size_t size = sizeof(calibration_value);
        err = nvs_get_blob(nvs_handle, storage_key, &calibration_value, &size);
        switch (err) {
            case ESP_OK:
                slots[i].calibration_factor = calibration_value;
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                slots[i].calibration_factor = calibration_value;
                nvs_set_blob(nvs_handle, storage_key, &calibration_value, size);
                nvs_commit(nvs_handle);
                break;
            default:
                ESP_LOGE(TAG, "Error reading NVS values!");
                nvs_close(nvs_handle);
                abort();
                break;
        }
    }
    nvs_close(nvs_handle);
}


void esp_now_message_received_isr(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    // Check if any data was received
    if (!data || len <= 0) return;

    message_t msg;

    // Create a message object for the queue
    if (len > sizeof(msg.data)) {
        ESP_LOGW(TAG, "Received ESP-NOW message that is too long for the buffer!");
        len = sizeof(msg.data);
    }

    memcpy(msg.data, data, len);
    msg.length = len;

    // Send to queue from ISR
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(esp_now_queue, &msg, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * Calculate the conversion factor for a slot
 * @param previous_val previous raw slot value
 * @param current_val current raw slot value (with known weight change applied)
 * @param tare_weight tare weight (known weight change)
 * @param output pointer to output
 * @return esp_err_T
 */
static esp_err_t calc_conversion_factor(double previous_val, double current_val, double tare_weight, double *output) {
    if (previous_val == current_val) {
        ESP_LOGW(TAG, "Loaded weight and zero weight are the same, can't compute conversion factor");
        return ESP_ERR_INVALID_ARG;
    } else {
        *output = (double)tare_weight / (current_val - previous_val);
        return ESP_OK;
    }
}


_Noreturn void process_esp_now_msg_task(void* pvParameters) {

    message_t msg;

    while (1) {

        // Get next message from the queue
        if (xQueueReceive(esp_now_queue, &msg, portMAX_DELAY)) {
            cJSON *json = cJSON_Parse(msg.data);

            // Make sure the slot_id and the weight_g exist
            if (cJSON_HasObjectItem(json, "slot_id") && cJSON_HasObjectItem(json, "weight_g")) {

                // Get the objects
                cJSON *slot_id_item = cJSON_GetObjectItem(json, "slot_id");
                cJSON *weight_g_item = cJSON_GetObjectItem(json, "weight_g");

                // Make sure they are numbers
                if (cJSON_IsNumber(slot_id_item) && cJSON_IsNumber(weight_g_item)) {
                    // Extract data
                    int slot_id = (int)cJSON_GetNumberValue(slot_id_item);
                    double weight_g = cJSON_GetNumberValue(weight_g_item);

                    // Make sure the slot is valid
                    if (slot_id < NUM_SLOTS) {
                        double output = 1.0;

                        double current_raw = slots[slot_id].current_raw;

                        // Calculate the conversion factor
                        esp_err_t err = calc_conversion_factor(slots[slot_id].calibration_previous_raw, current_raw, weight_g, &output);

                        if (err == ESP_OK) {
                            slots[slot_id].calibration_factor = output;
                            ESP_LOGI(TAG, "Set calibration factor to %.5f", output);
                            // Write to NVS
                            nvs_handle_t nvs_handle;
                            err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
                            if (err != ESP_OK) {
                                ESP_LOGW(TAG, "Failed to open NVS to write new calibration value to NVS");
                            } else {
                                char storage_key[30];
                                sprintf(storage_key, "slot_%d_cal", slot_id);
                                size_t size = sizeof(output);
                                err = nvs_set_blob(nvs_handle, storage_key, &output, size);
                                if (err != ESP_OK) {
                                    ESP_LOGW(TAG, "Failed to store new calibration value in NVS");
                                }
                                nvs_commit(nvs_handle);
                                nvs_close(nvs_handle);
                            }
                            slots[slot_id].calibration_previous_raw = current_raw;
                            ESP_LOGI(TAG, "Successfully tared");
                        }
                    } else {
                        ESP_LOGW(TAG, "Received slot ID exceeding max number of slots on this shelf");
                    }
                } else {
                    ESP_LOGW(TAG, "Received bad JSON packet over ESP-NOW: %s", msg.data);
                }
            } else {
                ESP_LOGW(TAG, "Calibration packet received without slot_id or weight_g fields");
            }
            // Free memory
            cJSON_Delete(json);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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
        if (json == NULL) {
            ESP_LOGE(TAG, "Failed to create JSON object");
            vTaskDelay(pdMS_TO_TICKS(WEIGHT_UPDATE_DELAY_MS));
            continue;
        }
        cJSON *shelves_array = cJSON_AddArrayToObject(json, "slots");
        if (shelves_array == NULL) {
            ESP_LOGE(TAG, "Failed to create JSON array");
            cJSON_Delete(json);
            vTaskDelay(pdMS_TO_TICKS(WEIGHT_UPDATE_DELAY_MS));
            continue;
        }

        // Read load cell data for each slot
        for (size_t i = 0; i < NUM_SLOTS; i++) {
            int32_t upper, lower;
            bool lower_ok = read_load_cell_data(slots[i].lower_load_cell, &lower) == ESP_OK;
            bool upper_ok = read_load_cell_data(slots[i].upper_load_cell, &upper) == ESP_OK;
            if (!lower_ok || !upper_ok) {
                // Error reading lower load cell, add null value to data
                if (!lower_ok && !upper_ok) {
                    ESP_LOGW(TAG, "Error reading both load cells in slot %d", i);
                } else if(!upper_ok) {
                    ESP_LOGW(TAG, "Error reading upper load cell in slot %d", i);
                } else {
                    ESP_LOGW(TAG, "Error reading lower load cell in slot %d", i);
                }
                cJSON_AddItemToArray(shelves_array, cJSON_CreateNull());
            } else {
                double raw_weight_total = (double)(upper + lower);
                double weight_value = raw_weight_total * slots[i].calibration_factor;
                if (i == 3) {
                    ESP_LOGD(TAG, "Slot %d weight value: %0.5f", i, weight_value);
                }
                cJSON *weight_value_json = cJSON_CreateNumber(weight_value);
                cJSON_AddItemToArray(shelves_array, weight_value_json);

                slots[i].current_raw = raw_weight_total;

            }
        }

        // Send JSON data to Atlas ESP32
        char *json_string = cJSON_PrintUnformatted(json);
        if (json_string != NULL) {
            size_t json_len = strlen(json_string);
            esp_now_send(atlas_mac, (uint8_t*)json_string, json_len);
            free(json_string);
        } else {
            ESP_LOGE(TAG, "Failed to serialize JSON. Free heap: %lu", esp_get_free_heap_size());
        }

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

    load_calibration();


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

    // Create ESP-NOW queue

    // Register callback for when a message is received over MQTT
    esp_now_queue = xQueueCreate(10, sizeof(message_t));
    esp_now_register_recv_cb(esp_now_message_received_isr);

    xTaskCreate(send_weights_task, "send_weights_task", 2048, NULL, 4, NULL);
    xTaskCreate(process_esp_now_msg_task, "process_esp_now_msg_task", 16384, NULL, 5, NULL);

}
