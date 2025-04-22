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
#include "secrets.h"
#include "esp_timer.h"
#include "esp_ws28xx.h"

// Pin definitions
#define DOOR_CONTROL_PIN 5
#define LEFT_DOOR_UPPER_SENSOR_PIN 11
#define LEFT_DOOR_LOWER_SENSOR_PIN 1
#define RIGHT_DOOR_UPPER_SENSOR_PIN 9
#define RIGHT_DOOR_LOWER_SENSOR_PIN 2
#define HATCH_CONTROL_PIN 7
#define HATCH_LEFT_SENSOR_PIN 3
#define HATCH_RIGHT_SENSOR_PIN 4

#define LED_PIN 37
#define LED_NUM 10

// MQTT definitions
#define MQTT_BROKER_URI "mqtt://bnbui.local"
#define DOOR_CONTROL_TOPIC "aux/control/doors"
#define HATCH_CONTROL_TOPIC "aux/control/hatch"
#define DOOR_STATUS_TOPIC "aux/status/doors"
#define HATCH_STATUS_TOPIC "aux/status/hatch"

#define DOOR_TRIGGER_DELAY_MS 100
#define HATCH_TRIGGER_DELAY_MS 100
#define CLOSED_MESSAGE_COOL_DOWN_MICRO_S 3000000

esp_mqtt_client_handle_t mqtt_client = NULL;
int64_t doors_closed_msg_last_send_micro_s = 0;

static const char *TAG = "doors";
static uint8_t led_state_off = 0;
CRGB* ws2812_buffer;

/**
 * Configure the pins for the doors
 */
void configure_pins() {
    gpio_set_level(DOOR_CONTROL_PIN, 1);
    gpio_set_level(HATCH_CONTROL_PIN, 1);
    // Configure the door control pins for the left and right doors
    gpio_config_t control_pin_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << DOOR_CONTROL_PIN) |
                            (1ULL << HATCH_CONTROL_PIN),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&control_pin_conf);

    // Configure the door sensor pins for the left and right doors
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
    // Set LED
    gpio_config_t led_pin_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << LED_PIN,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&led_pin_conf);
}


/**
 * Open the doors
 */
void open_doors() {
    // Set the open doors last send to prevent this from triggering the doors open
    doors_closed_msg_last_send_micro_s = esp_timer_get_time();
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
 * @return
 */
bool is_left_hatch_closed() {
    return !gpio_get_level(HATCH_LEFT_SENSOR_PIN);
}


/**
 * Check if the right hatch is closed
 * @return
 */
bool is_right_hatch_closed() {
    return !gpio_get_level(HATCH_RIGHT_SENSOR_PIN);
}


/**
 * Check if the hatch is closed
 * @return
 */
bool is_hatch_closed() {
    return is_left_hatch_closed() && is_right_hatch_closed();
}


/**
 * Publish that the doors are closed
 */
void publish_doors_closed() {
    char status_msg[7];
    sprintf(status_msg, "closed");
    if (mqtt_client) {
        // Check message cooldown time
        if (esp_timer_get_time() >= doors_closed_msg_last_send_micro_s + CLOSED_MESSAGE_COOL_DOWN_MICRO_S) {
            esp_mqtt_client_publish(mqtt_client, DOOR_STATUS_TOPIC, status_msg, 0, 1, 0);
            doors_closed_msg_last_send_micro_s = esp_timer_get_time();
        }
    }
}


/**
 * Publish that the hatches are closed
 */
void publish_hatch_closed() {
    char status_msg[7];
    sprintf(status_msg, "closed");
    if (mqtt_client) {
        esp_mqtt_client_publish(mqtt_client, HATCH_STATUS_TOPIC, status_msg, 0, 1, 0);
    }
}


/**
 * Function for a FreeRTOS task to constantly publish the door status.
 * @param pvParameters
 */
_Noreturn void publish_door_status_task(void *pvParameters) {
    bool doors_previously_closed = false; // Store previous state
    bool hatch_previously_closed = false; // Store the previous state
    while (1) {
        // Check doors status
        if (are_doors_closed()) {
            if (!doors_previously_closed) {
                // Publish message if doors are now closed and were not closed
                // previously
                doors_previously_closed = true;
                publish_doors_closed();
            }
        } else {
            doors_previously_closed = false;
        }
        // Check hatch status
        if (is_hatch_closed()) {
            if (!hatch_previously_closed) {
                // Publish message of hatch is now closed and was not closed
                // previously
                hatch_previously_closed = true;
                publish_hatch_closed();
            }
        } else {
            hatch_previously_closed = false;
        }
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
            esp_mqtt_client_subscribe(client, HATCH_CONTROL_TOPIC, 0);
            break;
        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, DOOR_CONTROL_TOPIC, event->topic_len) == 0) {
                if (strncmp(event->data, "open", event->data_len) == 0) {
                    open_doors();
                }
            } else if (strncmp(event->topic, HATCH_CONTROL_TOPIC, event->topic_len) == 0) {
                if (strncmp(event->data, "open", event->data_len) == 0) {
                    open_hatch();
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
 * Blink the LED
 */
void blink_led(void) {
    for(int i = 0; i < LED_NUM; i++) {
        if (led_state_off) ws2812_buffer[i] = (CRGB){.r=0, .g=0, .b=0};
        else ws2812_buffer[i] = (CRGB){.r=50, .g=0, .b=0};
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_update());
}
_Noreturn void blink_led_task(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "Turning the LED strip %s!", led_state_off == true ? "ON" : "OFF");
        blink_led();
        led_state_off = !led_state_off;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


/**
 * Main function, runs once on app start.
 */
void app_main(void)
{
    // Configure all GPIO pins
    configure_pins();

    // Get Wifi SSID and password
    const char *wifi_ssid = WIFI_SSID;
    const char *wifi_pass = WIFI_PASS;

    // Initialize Wifi
    nvs_flash_init();
    wifi_connection(wifi_ssid, wifi_pass);
    // Initialize the LED strip
    ESP_ERROR_CHECK(ws28xx_init(LED_PIN, WS2812B, LED_NUM, &ws2812_buffer));

    // Start the MQTT app
    mqtt_app_start();

    // Create the door status publishing task
    xTaskCreate(&publish_door_status_task, "publish_door_status_task", 2048, NULL, 5, NULL);
    xTaskCreate(&blink_led_task, "blink_led_task", 2048, NULL, 5, NULL);
}
