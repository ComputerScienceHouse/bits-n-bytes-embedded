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

// MQTT definitions
#define MQTT_BROKER_URI "mqtt://bnbui.local"
#define SHELF_DATA_TOPIC "shelf/data"

#define WEIGHT_POLLING_DELAY_MS 1000
#define LOAD_CELL_WAIT_TIMEOUT_MS 125
#define LOAD_CELL_SCALING_FACTOR 1

// Pin definitions
#define LED_PIN 15
#define LC_CLOCK_PIN 2
#define SLOT_0_UPPER_PIN 5
#define SLOT_0_LOWER_PIN 7
#define SLOT_1_UPPER_PIN 11
#define SLOT_1_LOWER_PIN 9
#define SLOT_2_UPPER_PIN 33
#define SLOT_2_LOWER_PIN 18
#define SLOT_3_UPPER_PIN 35
#define SLOT_3_LOWER_PIN 37
hx711_t slot_0_upper;
hx711_t slot_0_lower;
hx711_t slot_1_upper;
hx711_t slot_1_lower;
hx711_t slot_2_upper;
hx711_t slot_2_lower;
hx711_t slot_3_upper;
hx711_t slot_3_lower;

esp_mqtt_client_handle_t mqtt_client = NULL;

static char mac_address_str[18];

/**
 * Configure pins
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
 * @param load_cell_t
 * @param output
 */
void read_load_cell_data(hx711_t* load_cell_t, int32_t* output) {
    esp_err_t r = hx711_wait(load_cell_t, LOAD_CELL_WAIT_TIMEOUT_MS);
    if (r != ESP_OK) {
        printf("Load cell was not ready\n");
        return;
    }
    r = hx711_read_data(load_cell_t, output);
    if (r != ESP_OK) {
        printf("Unable to read load cell data\n");
    }
}


/**
 * Publish the value of the scales
 */
void publish_scale_values() {
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


    // Calculate the sum of the values for each scale, and multiply by the
    // scaling factor.
    double slot_values[4];
    slot_values[0] = (raw_values[0] + raw_values[1]) * LOAD_CELL_SCALING_FACTOR;
    slot_values[1] = (raw_values[2] + raw_values[3]) * LOAD_CELL_SCALING_FACTOR;
    slot_values[2] = (raw_values[4] + raw_values[5]) * LOAD_CELL_SCALING_FACTOR;
    slot_values[3] = (raw_values[6] + raw_values[7]) * LOAD_CELL_SCALING_FACTOR;

    // Copy the message into the buffer
    char payload_buffer[250];
    sprintf(
            payload_buffer,
            "{\n"
            "\tid: \"%s\"\n"
            "\tdata: [%lf, %lf, %lf, %lf]\n"
            "}\n",
            mac_address_str,
            slot_values[0],
            slot_values[1],
            slot_values[2],
            slot_values[3]
    );
    if (mqtt_client) {
        esp_mqtt_client_publish(mqtt_client, SHELF_DATA_TOPIC, payload_buffer, 0, 1, 0);
    }
}


/**
 * Task for publishing scale values. Can be created by xTaskCreate
 * @param pvParameters
 */
_Noreturn void publish_scale_values_task(void *pvParameters) {

    // Initialize all load cell
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

    // Continuously read the values
    while(1) {
        publish_scale_values();
        vTaskDelay(pdMS_TO_TICKS(WEIGHT_POLLING_DELAY_MS));
    }
}


/**
 * Callback to handle wifi events
 * @param event_handler_arg
 * @param event_base
 * @param event_id
 * @param event_data
 */
int retry_num = 0;
static void wifi_event_handler(void * event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        printf("Wifi connecting...\n");
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
        printf("Wifi connected\n");
        retry_num = 0;
        gpio_set_level(LED_PIN, 1);
    } else if(event_id == WIFI_EVENT_STA_DISCONNECTED) {
        printf("Wifi lost connection\n");
        esp_wifi_connect();
        printf("Trying to reconnect... (%d)\n", retry_num);
        // Flash LED based on the retry number
        if(retry_num % 2 == 0) {
            gpio_set_level(LED_PIN, 0);
        } else {
            gpio_set_level(LED_PIN, 1);
        }
        retry_num++;
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        printf("Wifi got IP\n");
    }
}


/**
 * Callback to handle incoming MQTT events
 * @param event The event to handle
 * @return
 */
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    // Extract the client from the event
    esp_mqtt_client_handle_t client = event->client;
    // Handle different event types
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            // On connection, subscribe to the topics
            break;
        case MQTT_EVENT_DATA:
            // Handle topics
            break;
        default:
            break;
    }
    return ESP_OK;
}


/**
 * Wrapper function for the mqtt_event_handler
 * @param handler_args
 * @param event_id
 * @param event_data
 */
static void mqtt_event_handler_wrapper(void *handler_args, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    mqtt_event_handler((esp_mqtt_event_handle_t) event_data);
}


/**
 * Start the MQTT portion of the app
 */
void mqtt_app_start() {
    // Create MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
            .broker.address.uri = MQTT_BROKER_URI
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    // Register callback for events
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler_wrapper, NULL);
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

/**
 * Connect to Wifi
 * @param wifi_ssid
 * @param wifi_pass
 */
void wifi_connection(const char* wifi_ssid, const char* wifi_pass) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_cfg_default = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_cfg_default);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_cfg = {
            .sta.ssid = "",
            .sta.password = "",
    };
    strcpy((char*)wifi_cfg.sta.ssid, wifi_ssid);
    strcpy((char*)wifi_cfg.sta.password, wifi_pass);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg);
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_connect();
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
 * Main function, runs once on app start.
 */
void app_main(void)
{

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Get Wifi SSID and password
    const char *wifi_ssid = WIFI_SSID;
    const char *wifi_pass = WIFI_PASS;

    // Initialize Wifi
    nvs_flash_init();
    wifi_connection(wifi_ssid, wifi_pass);

    // Store the mac address to send in MQTT messages
    store_mac_address();

    // Configure all GPIO pins
    configure_pins();

    // Start the MQTT app
    mqtt_app_start();

    // Create the shelf data publishing task
    xTaskCreate(&publish_scale_values_task, "publish_scale_values_task", 2048, NULL, 5, NULL);

}
