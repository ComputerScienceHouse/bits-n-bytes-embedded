#include <stdio.h>
#include <mqtt_client.h>
#include <driver/gpio.h>
#include "secrets.h"
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_wifi_default.h>
#include <esp_wifi.h>
#include <hx711.h>

// MQTT definitions
#define MQTT_BROKER_URI "mqtt://bnbui.local"
#define SHELF_DATA_TOPIC "shelf/data"

#define WEIGHT_POLLING_DELAY_MS 1000
#define LOAD_CELL_WAIT_TIMEOUT_MS 125
#define LOAD_CELL_SCALING_FACTOR 0.01

// Pin definitions
#define LED_PIN 15
hx711_t slot_0_upper = {
        .dout = GPIO_NUM_5,
        .pd_sck = GPIO_NUM_2,
        .gain = HX711_GAIN_A_128
};
hx711_t slot_0_lower = {
        .dout = GPIO_NUM_7,
        .pd_sck = GPIO_NUM_2,
        .gain = HX711_GAIN_A_128
};
hx711_t slot_1_upper = {
        .dout = GPIO_NUM_11,
        .pd_sck = GPIO_NUM_2,
        .gain = HX711_GAIN_A_128
};
hx711_t slot_1_lower = {
        .dout = GPIO_NUM_9,
        .pd_sck = GPIO_NUM_2,
        .gain = HX711_GAIN_A_128
};
hx711_t slot_2_upper = {
        .dout = GPIO_NUM_33,
        .pd_sck = GPIO_NUM_2,
        .gain = HX711_GAIN_A_128
};
hx711_t slot_2_lower = {
        .dout = GPIO_NUM_18,
        .pd_sck = GPIO_NUM_2,
        .gain = HX711_GAIN_A_128
};
hx711_t slot_3_upper = {
        .dout = GPIO_NUM_35,
        .pd_sck = GPIO_NUM_2,
        .gain = HX711_GAIN_A_128
};
hx711_t slot_3_lower = {
        .dout = GPIO_NUM_37,
        .pd_sck = GPIO_NUM_2,
        .gain = HX711_GAIN_A_128
};

esp_mqtt_client_handle_t mqtt_client = NULL;


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
 * Publish the value of the scales
 */
void publish_scale_values() {
    int32_t raw_values[8]; // Array for storing all values
    // Get shelf 0
    ESP_ERROR_CHECK(hx711_wait(&slot_0_upper, LOAD_CELL_WAIT_TIMEOUT_MS));
    ESP_ERROR_CHECK(hx711_read_data(&slot_0_upper, &raw_values[0]));
    ESP_ERROR_CHECK(hx711_wait(&slot_0_lower, LOAD_CELL_WAIT_TIMEOUT_MS));
    ESP_ERROR_CHECK(hx711_read_data(&slot_0_lower, &raw_values[1]));
    // Get shelf 1
    ESP_ERROR_CHECK(hx711_wait(&slot_1_upper, LOAD_CELL_WAIT_TIMEOUT_MS));
    ESP_ERROR_CHECK(hx711_read_data(&slot_1_upper, &raw_values[2]));
    ESP_ERROR_CHECK(hx711_wait(&slot_1_lower, LOAD_CELL_WAIT_TIMEOUT_MS));
    ESP_ERROR_CHECK(hx711_read_data(&slot_1_lower, &raw_values[3]));
    // Get shelf 2
    ESP_ERROR_CHECK(hx711_wait(&slot_2_upper, LOAD_CELL_WAIT_TIMEOUT_MS));
    ESP_ERROR_CHECK(hx711_read_data(&slot_2_upper, &raw_values[4]));
    ESP_ERROR_CHECK(hx711_wait(&slot_2_lower, LOAD_CELL_WAIT_TIMEOUT_MS));
    ESP_ERROR_CHECK(hx711_read_data(&slot_2_lower, &raw_values[5]));
    // Get shelf 3
    ESP_ERROR_CHECK(hx711_wait(&slot_3_upper, LOAD_CELL_WAIT_TIMEOUT_MS));
    ESP_ERROR_CHECK(hx711_read_data(&slot_3_upper, &raw_values[6]));
    ESP_ERROR_CHECK(hx711_wait(&slot_3_lower, LOAD_CELL_WAIT_TIMEOUT_MS));
    ESP_ERROR_CHECK(hx711_read_data(&slot_3_lower, &raw_values[7]));

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
            "\tid:\n"
            "\tdata:[%lf, %lf, %lf, %lf]\n"
            "}\n",
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
    while(1) {
        //publish_scale_values();
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(WEIGHT_POLLING_DELAY_MS));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
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
 * Main function, runs once on app start.
 */
void app_main(void)
{
    // Get Wifi SSID and password
    const char *wifi_ssid = WIFI_SSID;
    const char *wifi_pass = WIFI_PASS;

    // Initialize Wifi
    nvs_flash_init();
    wifi_connection(wifi_ssid, wifi_pass);

    // Configure all GPIO pins
    configure_pins();

    // Start the MQTT app
    //mqtt_app_start();

    // Create the shelf data publishing task
    xTaskCreate(&publish_scale_values_task, "publish_scale_values_task", 2048, NULL, 5, NULL);

}
