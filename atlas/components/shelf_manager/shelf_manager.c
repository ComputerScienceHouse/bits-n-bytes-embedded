#include <stdio.h>
#include "shelf_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "esp_timer.h"

#ifndef MAX_NUM_SHELVES
#define MAX_NUM_SHELVES 10
#endif

#ifndef MAC_ADDRESS_LENGTH
#define MAC_ADDRESS_LENGTH 6
#endif

// Delays and timeouts
#define SHELF_DISCONNECT_MS 3000

#define MAX_MUTEX_DELAY_MS 1000
#define MAX_MUTEX_DELAY_TICKS pdMS_TO_TICKS(MAX_MUTEX_DELAY_MS)

static const char* TAG = "shelf_manager";


/**
 * Struct defining what data is relevant to a shelf.
 * int64_t last_comms_mu_s: Last communication with this shelf, in MS
 * uint8_t mac_address[6]: Mac address of this shelf
 */
typedef struct {
    int64_t last_comms_mu_s;
    uint8_t mac_address[6];
} sm_shelf_t;

// Array of active shelves
static sm_shelf_t* active_shelves[MAX_NUM_SHELVES];
static size_t num_active_shelves = 0;
static SemaphoreHandle_t active_shelves_mutex = NULL;

static TaskHandle_t sm_task_handle = NULL;


/**
 * Add a new shelf.
 * @param mac_address
 * @return esp_err_t
 */
esp_err_t sm_add_shelf(uint8_t* mac_address, int64_t msg_recv_mu_s) {

    // Wait for access to the shelves array
    if (xSemaphoreTake(active_shelves_mutex, MAX_MUTEX_DELAY_TICKS)) {

        // Check if max number of shelves has been reached
        if (num_active_shelves < MAX_NUM_SHELVES) {
            // Good to add shelf

            // Allocate memory for the shelf
            sm_shelf_t* new_shelf = malloc(sizeof(sm_shelf_t));
            if (new_shelf == NULL) {
                ESP_LOGE(TAG, "Unable to allocate memory for new shelf");
                xSemaphoreGive(active_shelves_mutex);
                return ESP_ERR_NO_MEM;
            }
            // Set all fields
            memset(new_shelf, 0, sizeof(sm_shelf_t));
            new_shelf->last_comms_mu_s = msg_recv_mu_s;
            memcpy(new_shelf->mac_address, mac_address, MAC_ADDRESS_LENGTH);

            // Add new shelf pointer to list of shelves
            active_shelves[num_active_shelves] = new_shelf;
            num_active_shelves++;
        } else {
            // Hit max number of shelves
            ESP_LOGW(TAG, "Unable to add shelf: Max number of shelves (%d) hit!", MAX_NUM_SHELVES);
            xSemaphoreGive(active_shelves_mutex);
            return ESP_ERR_NO_MEM;
        }
        xSemaphoreGive(active_shelves_mutex);
    } else {
        // Didn't get mutex in time
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG, "Added shelf");
    return ESP_OK;
}


/**
 * Update the last time a shelf communicated.
 * @param mac_address
 * @param msg_recv_mu_
 * @return
 */
esp_err_t sm_update_shelf_time(uint8_t* mac_address, int64_t msg_recv_mu_s) {
    if (xSemaphoreTake(active_shelves_mutex, MAX_MUTEX_DELAY_TICKS)) {
        // Find the correct shelf
        bool found = false;
        for (size_t i = 0; i < num_active_shelves; i++) {
            // Check if this is the right shelf by mac address
            if (memcmp(active_shelves[i]->mac_address, mac_address, MAC_ADDRESS_LENGTH) == 0) {
                // Update time
                active_shelves[i]->last_comms_mu_s = msg_recv_mu_s;
                found = true;
                break;
            }
        }
        xSemaphoreGive(active_shelves_mutex);
        // Check if right shelf was found
        if (!found) {
            return ESP_ERR_NOT_FOUND;
        }
    } else {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}


/**
 * Check if a shelf is connected
 * @param mac_address
 * @return
 */
bool sm_is_shelf_connected(uint8_t* mac_address) {
    if (xSemaphoreTake(active_shelves_mutex, MAX_MUTEX_DELAY_TICKS)) {
        // Iterate through all shelves searching for this mac address
        for (size_t i = 0; i < num_active_shelves; i++) {
            // Check if this shelf has the right mac address
            if (memcmp(active_shelves[i]->mac_address, mac_address, MAC_ADDRESS_LENGTH) == 0) {
                xSemaphoreGive(active_shelves_mutex);
                return true;
            }
        }
        xSemaphoreGive(active_shelves_mutex);
    }
    return false;
}

/**
 * Get the mac addresses of all active shelves.
 * @param output allocated array of uint8_t*. Must be freed after use, along
 * with all uint8_t pointers contained within.
 * @return esp_err_t
 */
esp_err_t sm_get_all_active_shelves_mac_addresses(uint8_t*** output, size_t *num_shelves) {

    if (xSemaphoreTake(active_shelves_mutex, MAX_MUTEX_DELAY_TICKS)) {

        if (num_active_shelves == 0) {
            *num_shelves = 0;
            xSemaphoreGive(active_shelves_mutex);
            return ESP_OK;
        }

        *output = malloc(num_active_shelves * sizeof(uint8_t*));

        if (*output == NULL) {
            xSemaphoreGive(active_shelves_mutex);
            return ESP_ERR_NO_MEM;
        }

        for (size_t i = 0; i < num_active_shelves; i++) {
            (*output)[i] = malloc(MAC_ADDRESS_LENGTH);
            if ((*output)[i] == NULL) {
                // Clean up on failure
                for (size_t j = 0; j < i; j++) {
                    free((*output)[j]);
                }
                free(*output);
                xSemaphoreGive(active_shelves_mutex);
                return ESP_ERR_NO_MEM;
            }
            memcpy((*output)[i], active_shelves[i]->mac_address, MAC_ADDRESS_LENGTH);
        }
        *num_shelves = num_active_shelves;
        xSemaphoreGive(active_shelves_mutex);
        return ESP_OK;
    } else {
        return ESP_ERR_TIMEOUT;
    }
}


/**
 * Remove a shelf. NOTE: This is ONLY to be used internally by this library, as
 * this function DOES NOT handle locks. The lock must be taken by the caller.
 * @param mac_address The mac address of the shelf to remove.
 * @return esp_err_t
 */
static esp_err_t sm_internal_remove_shelf(uint8_t *mac_address) {
    // Iterate through all shelves to find the one to remove
    bool found = false;
    size_t remove_i = 0;
    for (size_t i = 0; i < num_active_shelves; i++) {
        if (memcmp(active_shelves[i]->mac_address, mac_address, MAC_ADDRESS_LENGTH) == 0) {
            // Remove shelf
            free(active_shelves[i]);
            active_shelves[i] = NULL;
            num_active_shelves--;
            // Signal that removal happened
            found = true;
            remove_i = i;
            break;
        }
    }
    if (found) {
        // Shift all shelves that come after the one that was removed
        for (size_t i = remove_i; i < num_active_shelves; i++) {
            active_shelves[i] = active_shelves[i + 1];
        }
    } else {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Removed shelf");
    return ESP_OK;
}


/**
 * Task to remove old shelves automatically.
 */
static _Noreturn void sm_task() {
    while (1) {

        if (xSemaphoreTake(active_shelves_mutex, portMAX_DELAY)) {
            int64_t now_mu_s = esp_timer_get_time();
            // Iterate through all shelves
            for (size_t i = 0; i < num_active_shelves; i++) {
                // Check if the shelf has not said anything for too long
                if ((now_mu_s - active_shelves[i]->last_comms_mu_s) / 1000 > SHELF_DISCONNECT_MS) {
                    // Time since shelf last said anything has exceeded max delay. Remove shelf
                    sm_internal_remove_shelf(active_shelves[i]->mac_address);
                }
            }
            xSemaphoreGive(active_shelves_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/**
 * Initialize the shelf manager and start the shelf manager task.
 * @return
 */
esp_err_t sm_init() {
    active_shelves_mutex = xSemaphoreCreateMutex();

    BaseType_t x_returned = xTaskCreate(sm_task, "shelf_manager_task", 4096, NULL, 5, &sm_task_handle);

    if (x_returned != pdPASS) {
        ESP_LOGE(TAG, "Failed to start shelf manager task!");
        return ESP_FAIL;
    }
    return ESP_OK;
}


/**
 * Stop the shelf manager.
 * @return
 */
esp_err_t sm_stop() {
    vTaskDelete(sm_task_handle);
    return ESP_OK;
}
