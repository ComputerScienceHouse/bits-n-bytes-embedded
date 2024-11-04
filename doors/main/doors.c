#include <sys/cdefs.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "mqtt_client.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"

// Pin definitions
#define LEFT_DOOR_CONTROL_PIN 12
#define LEFT_DOOR_SENSOR_PIN 13
#define RIGHT_DOOR_CONTROL_PIN 14
#define RIGHT_DOOR_SENSOR_PIN 15

// MQTT definitions
#define MQTT_BROKER_URI ""
#define DOOR_CONTROL_TOPIC "aux/control/doors"
#define DOOR_STATUS_TOPIC "aux/status/doors"

#define PUBLISH_STATUS_DELAY_MS 1000
#define DOOR_TRIGGER_DELAY_MS 1000

esp_mqtt_client_handle_t mqtt_client = NULL;

/**
 * Configure the pins for the doors
 */
void configure_pins() {
    // Configure the door control pins for the left and right doors
    gpio_config_t door_control_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << LEFT_DOOR_CONTROL_PIN) |
                    (1ULL << RIGHT_DOOR_CONTROL_PIN),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&door_control_conf);
    // Configure the door sensor pins for the left and right doors
    gpio_config_t door_sensor_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << LEFT_DOOR_SENSOR_PIN) |
                    (1ULL << RIGHT_DOOR_SENSOR_PIN),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&door_sensor_conf);
}


/**
 * Open the doors
 */
void open_doors() {
    gpio_set_level(LEFT_DOOR_CONTROL_PIN, 1);
    gpio_set_level(RIGHT_DOOR_CONTROL_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(DOOR_TRIGGER_DELAY_MS));
    gpio_set_level(LEFT_DOOR_CONTROL_PIN, 0);
    gpio_set_level(RIGHT_DOOR_CONTROL_PIN, 0);
}


/**
 * Check if the left door is closed
 * @return boolean
 */
bool is_left_door_closed() {
    return gpio_get_level(LEFT_DOOR_SENSOR_PIN);
}


/**
 * Check if the right door is closed
 * @return boolean
 */
bool is_right_door_closed() {
    return gpio_get_level(RIGHT_DOOR_SENSOR_PIN);
}


/**
 * Check if both doors are closeds
 *
 * @return boolean
 */
bool are_doors_closed() {
    return is_left_door_closed() && is_right_door_closed();
}


/**
 * Publish the door status
 */
void publish_door_status() {
    char status_msg[7];
    if (are_doors_closed()) {
        sprintf(status_msg, "closed");
    } else {
        sprintf(status_msg, "open");
    }
    if (mqtt_client) {
        esp_mqtt_client_publish(mqtt_client, DOOR_STATUS_TOPIC, status_msg, 0, 1, 0);
    }
}


/**
 * Function for a FreeRTOS task to constantly publish the door status.
 * @param pvParameters
 */
_Noreturn void publish_door_status_task(void *pvParameters) {
    while (1) {
        publish_door_status();
        vTaskDelay(pdMS_TO_TICKS(PUBLISH_STATUS_DELAY_MS));
    }
}

/***
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
            // On connection, subscribe to the door control topic
            esp_mqtt_client_subscribe(client, DOOR_CONTROL_TOPIC, 0);
            break;
        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, DOOR_CONTROL_TOPIC, event->topic_len) == 0) {
                if (strncmp(event->data, "open", event->data_len) == 0) {
                    open_doors();
                }
            }
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
    // Start the MQTT client
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}


int retry_num = 0;
static void wifi_event_handler(void * event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        printf("Wifi connecting...\n");
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
        printf("Wifi connected\n");
        retry_num = 0;
    } else if(event_id == WIFI_EVENT_STA_DISCONNECTED) {
        printf("Wifi lost connection\n");
        if(retry_num < 5) {
            esp_wifi_connect();
            retry_num++;
            printf("Trying to reconnect... (%d)\n", retry_num);
        }
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        printf("Wifi got IP\n");
    }
}

/**
 * Connect to Wifi
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
    const char *wifi_ssid = getenv("ESP_WIFI_SSID");
    if(wifi_ssid == NULL) {
        printf("Wifi SSID not defined in env!");
    }
    const char *wifi_pass = getenv("ESP_WIFI_PASS");
    if(wifi_pass == NULL) {
        printf("Wifi password not defined in env!");
    }
    if (wifi_ssid == NULL || wifi_pass == NULL) {
        exit(1);
    }

    // Initialize Wifi
    nvs_flash_init();
    wifi_connection(wifi_ssid, wifi_pass);

    // Configure all GPIO pins
    configure_pins();

    // Start the MQTT app
    mqtt_app_start();

    // Create the door status publishing task
    xTaskCreate(&publish_door_status_task, "publish_door_status_task", 2048, NULL, 5, NULL);
}
