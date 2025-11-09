#include "esp_log.h"
#include "freertos/FreeRTOS.h"

/**
 * Add a new shelf.
 * @param mac_address
 * @return esp_err_t
 */
esp_err_t sm_add_shelf(uint8_t* mac_address, int64_t msg_recv_mu_s);


/**
 * Update the last time a shelf communicated.
 * @param mac_address
 * @param msg_recv_mu_
 * @return
 */
esp_err_t sm_update_shelf_time(uint8_t* mac_address, int64_t msg_recv_mu_s);


/**
 * Get the mac addresses of all active shelves.
 * @param output allocated array of uint8_t*. Must be freed after use, along
 * with all uint8_t pointers contained within.
 * @return esp_err_t
 */
esp_err_t sm_get_all_active_shelves_mac_addresses(uint8_t*** output, size_t *num_shelves);


/**
 * Check if a shelf is connected
 * @param mac_address
 * @return
 */
bool sm_is_shelf_connected(uint8_t* mac_address);


/**
 * Initialize the shelf manager and start the shelf manager task.
 * @return
 */
esp_err_t sm_init();


/**
 * Stop the shelf manager.
 * @return
 */
esp_err_t sm_stop();
