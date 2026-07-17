#include "bnb_ack.h"

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "bnb_ack";

#define BNB_ACK_MUTEX_TIMEOUT_MS 1000

// Bound on how many transmit/deliver/failed callbacks a single retry sweep
// will fire before yielding. Anything past this catches up on the next
// sweep (BNB_ACK_SWEEP_INTERVAL_MS later) since entries left un-swept keep
// their old last_sent_us and stay due. This keeps the sweep's stack usage
// bounded regardless of max_peers.
#define BNB_ACK_RETRY_ACTIONS_PER_SWEEP 8

typedef struct {
    bool in_use;
    uint16_t seq_id;
    bnb_msg_type_t type;
    uint8_t envelope[BNB_PROTOCOL_MAX_ENVELOPE_LEN];
    size_t envelope_len;
    int64_t last_sent_us;
    uint8_t retry_count;
} bnb_ack_pending_send_t;

typedef struct {
    bool in_use;
    uint16_t seq_id;                                      // dedup key for inbound data messages
    uint8_t ack_envelope[BNB_PROTOCOL_MAX_ENVELOPE_LEN];   // pre-built BNB_MSG_ACK, resent verbatim
    size_t ack_envelope_len;
    int64_t last_sent_us;
    uint8_t retry_count;
} bnb_ack_pending_ack_t;

typedef struct {
    bool in_use;
    uint8_t peer_mac[6];
    uint16_t next_seq_id;
    bnb_ack_pending_send_t pending_send[BNB_ACK_MAX_PENDING_SEND];
    bnb_ack_pending_ack_t pending_ack[BNB_ACK_MAX_PENDING_ACK];
} bnb_ack_peer_t;

typedef struct {
    bool is_failed; // true -> fire failed_fn_g, false -> fire transmit_fn_g
    uint8_t peer_mac[6];
    bnb_msg_type_t type;
    uint16_t seq_id;
    uint8_t envelope[BNB_PROTOCOL_MAX_ENVELOPE_LEN];
    size_t envelope_len;
} bnb_ack_retry_action_t;

static bnb_ack_peer_t *peers = NULL;
static size_t max_peers_g = 0;
static SemaphoreHandle_t ack_mutex = NULL;
static TaskHandle_t retry_task_handle = NULL;

static bnb_ack_transmit_fn_t transmit_fn_g = NULL;
static bnb_ack_delivered_fn_t delivered_fn_g = NULL;
static bnb_ack_failed_fn_t failed_fn_g = NULL;
static void *ctx_g = NULL;

static _Noreturn void bnb_ack_retry_task(void *pvParameter);

static esp_err_t take_lock(void) {
    if (xSemaphoreTake(ack_mutex, pdMS_TO_TICKS(BNB_ACK_MUTEX_TIMEOUT_MS))) {
        return ESP_OK;
    }
    ESP_LOGW(TAG, "Failed to acquire bnb_ack lock in time");
    return ESP_ERR_TIMEOUT;
}

// Must be called with ack_mutex held.
static bnb_ack_peer_t *find_or_create_peer(const uint8_t mac[6]) {
    for (size_t i = 0; i < max_peers_g; i++) {
        if (peers[i].in_use && memcmp(peers[i].peer_mac, mac, 6) == 0) {
            return &peers[i];
        }
    }
    for (size_t i = 0; i < max_peers_g; i++) {
        if (!peers[i].in_use) {
            memset(&peers[i], 0, sizeof(bnb_ack_peer_t));
            peers[i].in_use = true;
            memcpy(peers[i].peer_mac, mac, 6);
            peers[i].next_seq_id = 1;
            return &peers[i];
        }
    }
    return NULL;
}

static esp_err_t build_ack_envelope(bnb_msg_type_t ack_type, uint16_t acked_seq_id,
                                     uint8_t *out_buf, size_t out_buf_cap, size_t *out_len) {
    bnb_ack_payload_t ack_payload = { .seq_id = acked_seq_id };
    return bnb_protocol_encode(ack_type, acked_seq_id, &ack_payload, sizeof(ack_payload),
                                out_buf, out_buf_cap, out_len);
}

esp_err_t bnb_ack_init(size_t max_peers, bnb_ack_transmit_fn_t transmit_fn,
                        bnb_ack_delivered_fn_t delivered_fn, bnb_ack_failed_fn_t failed_fn, void *ctx) {
    if (max_peers == 0 || transmit_fn == NULL || delivered_fn == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    peers = calloc(max_peers, sizeof(bnb_ack_peer_t));
    if (peers == NULL) {
        return ESP_ERR_NO_MEM;
    }
    max_peers_g = max_peers;
    transmit_fn_g = transmit_fn;
    delivered_fn_g = delivered_fn;
    failed_fn_g = failed_fn;
    ctx_g = ctx;

    ack_mutex = xSemaphoreCreateMutex();
    if (ack_mutex == NULL) {
        free(peers);
        peers = NULL;
        return ESP_ERR_NO_MEM;
    }

    BaseType_t created = xTaskCreate(bnb_ack_retry_task, "bnb_ack_retry_task", 8192, NULL, 5, &retry_task_handle);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "Failed to start bnb_ack retry task");
        vSemaphoreDelete(ack_mutex);
        ack_mutex = NULL;
        free(peers);
        peers = NULL;
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bnb_ack_send_reliable(const uint8_t peer_mac[6], bnb_msg_type_t type,
                                 const void *payload, size_t payload_len) {
    if (peer_mac == NULL || (payload == NULL && payload_len > 0)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload_len > BNB_PROTOCOL_MAX_PAYLOAD_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t lock_err = take_lock();
    if (lock_err != ESP_OK) {
        return lock_err;
    }

    bnb_ack_peer_t *peer = find_or_create_peer(peer_mac);
    if (peer == NULL) {
        xSemaphoreGive(ack_mutex);
        ESP_LOGW(TAG, "Unable to track new peer for reliable send: max_peers reached");
        return ESP_ERR_NO_MEM;
    }

    bnb_ack_pending_send_t *slot = NULL;
    for (size_t i = 0; i < BNB_ACK_MAX_PENDING_SEND; i++) {
        if (!peer->pending_send[i].in_use) {
            slot = &peer->pending_send[i];
            break;
        }
    }
    if (slot == NULL) {
        xSemaphoreGive(ack_mutex);
        ESP_LOGW(TAG, "Pending-send table full for peer, dropping reliable send of type %d", type);
        return ESP_ERR_NO_MEM;
    }

    uint16_t seq_id = peer->next_seq_id++;
    esp_err_t encode_err = bnb_protocol_encode(type, seq_id, payload, payload_len,
                                                slot->envelope, sizeof(slot->envelope), &slot->envelope_len);
    if (encode_err != ESP_OK) {
        xSemaphoreGive(ack_mutex);
        return encode_err;
    }

    slot->in_use = true;
    slot->seq_id = seq_id;
    slot->type = type;
    slot->retry_count = 0;
    slot->last_sent_us = esp_timer_get_time();

    uint8_t envelope_copy[BNB_PROTOCOL_MAX_ENVELOPE_LEN];
    size_t envelope_len_copy = slot->envelope_len;
    memcpy(envelope_copy, slot->envelope, envelope_len_copy);
    uint8_t mac_copy[6];
    memcpy(mac_copy, peer_mac, 6);

    xSemaphoreGive(ack_mutex);

    transmit_fn_g(mac_copy, envelope_copy, envelope_len_copy, ctx_g);
    return ESP_OK;
}

void bnb_ack_handle_incoming(const uint8_t peer_mac[6], const uint8_t *raw_envelope, size_t raw_len) {
    if (peer_mac == NULL || raw_envelope == NULL) {
        return;
    }

    bnb_msg_type_t msg_type;
    uint16_t seq_id;
    uint8_t payload[BNB_PROTOCOL_MAX_PAYLOAD_LEN];
    size_t payload_len;
    esp_err_t decode_err = bnb_protocol_decode(raw_envelope, raw_len, &msg_type, &seq_id,
                                                payload, sizeof(payload), &payload_len);
    if (decode_err != ESP_OK) {
        ESP_LOGW(TAG, "Dropping ESP-NOW message that failed to decode: %s", esp_err_to_name(decode_err));
        return;
    }

    esp_err_t lock_err = take_lock();
    if (lock_err != ESP_OK) {
        return;
    }

    bnb_ack_peer_t *peer = find_or_create_peer(peer_mac);
    if (peer == NULL) {
        xSemaphoreGive(ack_mutex);
        ESP_LOGW(TAG, "Unable to track new peer for incoming message: max_peers reached");
        return;
    }

    bool do_deliver = false;
    bool do_transmit = false;
    uint8_t transmit_envelope[BNB_PROTOCOL_MAX_ENVELOPE_LEN];
    size_t transmit_envelope_len = 0;
    uint8_t deliver_payload[BNB_PROTOCOL_MAX_PAYLOAD_LEN];
    size_t deliver_payload_len = 0;
    uint8_t mac_copy[6];
    memcpy(mac_copy, peer_mac, 6);

    if (msg_type == BNB_MSG_ACK) {
        if (payload_len < sizeof(bnb_ack_payload_t)) {
            ESP_LOGW(TAG, "Dropping malformed BNB_MSG_ACK (short payload)");
            xSemaphoreGive(ack_mutex);
            return;
        }
        bnb_ack_payload_t ack_payload;
        memcpy(&ack_payload, payload, sizeof(ack_payload));

        for (size_t i = 0; i < BNB_ACK_MAX_PENDING_SEND; i++) {
            bnb_ack_pending_send_t *s = &peer->pending_send[i];
            if (s->in_use && s->seq_id == ack_payload.seq_id) {
                s->in_use = false;
                break;
            }
        }
        // Unconditional reply, even if the entry above was already gone --
        // a duplicate ACK means the sender never saw our previous ACK_ACK.
        esp_err_t build_err = build_ack_envelope(BNB_MSG_ACK_ACK, ack_payload.seq_id,
                                                  transmit_envelope, sizeof(transmit_envelope), &transmit_envelope_len);
        do_transmit = (build_err == ESP_OK);

    } else if (msg_type == BNB_MSG_ACK_ACK) {
        if (payload_len < sizeof(bnb_ack_payload_t)) {
            ESP_LOGW(TAG, "Dropping malformed BNB_MSG_ACK_ACK (short payload)");
            xSemaphoreGive(ack_mutex);
            return;
        }
        bnb_ack_payload_t ack_payload;
        memcpy(&ack_payload, payload, sizeof(ack_payload));

        for (size_t i = 0; i < BNB_ACK_MAX_PENDING_ACK; i++) {
            bnb_ack_pending_ack_t *a = &peer->pending_ack[i];
            if (a->in_use && a->seq_id == ack_payload.seq_id) {
                a->in_use = false;
                break;
            }
        }
        // Terminal -- no further action.

    } else {
        // Data message type (BNB_MSG_SHELF_STATUS / CALIBRATION / POSITION_SET).
        bnb_ack_pending_ack_t *existing = NULL;
        for (size_t i = 0; i < BNB_ACK_MAX_PENDING_ACK; i++) {
            if (peer->pending_ack[i].in_use && peer->pending_ack[i].seq_id == seq_id) {
                existing = &peer->pending_ack[i];
                break;
            }
        }

        if (existing != NULL) {
            // Retransmit of a message we've already delivered -- resend the
            // stored ACK, do not re-deliver to the app.
            memcpy(transmit_envelope, existing->ack_envelope, existing->ack_envelope_len);
            transmit_envelope_len = existing->ack_envelope_len;
            do_transmit = true;
        } else {
            bnb_ack_pending_ack_t *slot = NULL;
            for (size_t i = 0; i < BNB_ACK_MAX_PENDING_ACK; i++) {
                if (!peer->pending_ack[i].in_use) {
                    slot = &peer->pending_ack[i];
                    break;
                }
            }
            if (slot == NULL) {
                // Table full -- evict the oldest entry rather than block.
                size_t oldest_i = 0;
                int64_t oldest_time = peer->pending_ack[0].last_sent_us;
                for (size_t i = 1; i < BNB_ACK_MAX_PENDING_ACK; i++) {
                    if (peer->pending_ack[i].last_sent_us < oldest_time) {
                        oldest_time = peer->pending_ack[i].last_sent_us;
                        oldest_i = i;
                    }
                }
                slot = &peer->pending_ack[oldest_i];
                ESP_LOGW(TAG, "Pending-ack table full, evicting oldest entry (seq %u)", slot->seq_id);
            }

            esp_err_t build_err = build_ack_envelope(BNB_MSG_ACK, seq_id, slot->ack_envelope,
                                                      sizeof(slot->ack_envelope), &slot->ack_envelope_len);
            if (build_err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to build ACK envelope: %s", esp_err_to_name(build_err));
                xSemaphoreGive(ack_mutex);
                return;
            }
            slot->in_use = true;
            slot->seq_id = seq_id;
            slot->retry_count = 0;
            slot->last_sent_us = esp_timer_get_time();

            memcpy(transmit_envelope, slot->ack_envelope, slot->ack_envelope_len);
            transmit_envelope_len = slot->ack_envelope_len;
            do_transmit = true;

            memcpy(deliver_payload, payload, payload_len);
            deliver_payload_len = payload_len;
            do_deliver = true;
        }
    }

    xSemaphoreGive(ack_mutex);

    // Both callbacks run outside the lock. delivered_fn runs first so app
    // processing happens-before the ACK goes out.
    if (do_deliver) {
        delivered_fn_g(mac_copy, msg_type, deliver_payload, deliver_payload_len, ctx_g);
    }
    if (do_transmit) {
        transmit_fn_g(mac_copy, transmit_envelope, transmit_envelope_len, ctx_g);
    }
}

static _Noreturn void bnb_ack_retry_task(void *pvParameter) {
    (void)pvParameter;

    while (1) {
        bnb_ack_retry_action_t actions[BNB_ACK_RETRY_ACTIONS_PER_SWEEP];
        size_t num_actions = 0;

        if (xSemaphoreTake(ack_mutex, pdMS_TO_TICKS(BNB_ACK_MUTEX_TIMEOUT_MS))) {
            int64_t now_us = esp_timer_get_time();

            for (size_t p = 0; p < max_peers_g && num_actions < BNB_ACK_RETRY_ACTIONS_PER_SWEEP; p++) {
                bnb_ack_peer_t *peer = &peers[p];
                if (!peer->in_use) {
                    continue;
                }

                for (size_t i = 0; i < BNB_ACK_MAX_PENDING_SEND && num_actions < BNB_ACK_RETRY_ACTIONS_PER_SWEEP; i++) {
                    bnb_ack_pending_send_t *s = &peer->pending_send[i];
                    if (!s->in_use || (now_us - s->last_sent_us) / 1000 < BNB_ACK_RETRY_INTERVAL_MS) {
                        continue;
                    }
                    bnb_ack_retry_action_t *action = &actions[num_actions++];
                    memcpy(action->peer_mac, peer->peer_mac, 6);
                    if (s->retry_count >= BNB_ACK_MAX_SEND_RETRIES) {
                        ESP_LOGE(TAG, "Giving up on reliable delivery of type %d seq %u after %d retries",
                                 s->type, s->seq_id, s->retry_count);
                        action->is_failed = true;
                        action->type = s->type;
                        action->seq_id = s->seq_id;
                        s->in_use = false;
                    } else {
                        action->is_failed = false;
                        action->envelope_len = s->envelope_len;
                        memcpy(action->envelope, s->envelope, s->envelope_len);
                        s->retry_count++;
                        s->last_sent_us = now_us;
                    }
                }

                for (size_t i = 0; i < BNB_ACK_MAX_PENDING_ACK && num_actions < BNB_ACK_RETRY_ACTIONS_PER_SWEEP; i++) {
                    bnb_ack_pending_ack_t *a = &peer->pending_ack[i];
                    if (!a->in_use || (now_us - a->last_sent_us) / 1000 < BNB_ACK_RETRY_INTERVAL_MS) {
                        continue;
                    }
                    if (a->retry_count >= BNB_ACK_MAX_ACK_RETRIES) {
                        ESP_LOGW(TAG, "Giving up on resending ACK for seq %u after %d retries", a->seq_id, a->retry_count);
                        a->in_use = false;
                        continue;
                    }
                    bnb_ack_retry_action_t *action = &actions[num_actions++];
                    memcpy(action->peer_mac, peer->peer_mac, 6);
                    action->is_failed = false;
                    action->envelope_len = a->ack_envelope_len;
                    memcpy(action->envelope, a->ack_envelope, a->ack_envelope_len);
                    a->retry_count++;
                    a->last_sent_us = now_us;
                }
            }

            xSemaphoreGive(ack_mutex);
        } else {
            ESP_LOGW(TAG, "bnb_ack retry sweep failed to acquire lock");
        }

        for (size_t i = 0; i < num_actions; i++) {
            if (actions[i].is_failed) {
                if (failed_fn_g != NULL) {
                    failed_fn_g(actions[i].peer_mac, actions[i].type, actions[i].seq_id, ctx_g);
                }
            } else {
                transmit_fn_g(actions[i].peer_mac, actions[i].envelope, actions[i].envelope_len, ctx_g);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BNB_ACK_SWEEP_INTERVAL_MS));
    }
}
