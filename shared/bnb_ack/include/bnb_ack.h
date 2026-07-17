/**
 * File: bnb_ack.h
 *
 * Purpose: QoS2-style reliable delivery on top of the bnb_protocol wire
 * envelope. Owns encode/decode for every message that goes over ESP-NOW
 * (data messages and its own internal BNB_MSG_ACK / BNB_MSG_ACK_ACK types) —
 * callers never touch bnb_protocol_encode/decode directly, they call
 * bnb_ack_send_reliable() to send and bnb_ack_handle_incoming() to feed in
 * raw received bytes.
 *
 * Handshake for one message, seq=S, sender A -> receiver B:
 *   1. A calls bnb_ack_send_reliable(); A retries the DATA envelope until
 *      it sees an ACK(seq=S) come back.
 *   2. B receives DATA(seq=S): if new, delivers it to the app exactly once
 *      via delivered_fn, then sends ACK(seq=S) and keeps retrying that ACK
 *      until it sees ACK_ACK(seq=S). If DATA(seq=S) arrives again (A never
 *      saw the first ACK), B just resends the stored ACK without
 *      re-delivering to the app.
 *   3. A receives ACK(seq=S): stops retrying DATA, and unconditionally
 *      replies with ACK_ACK(seq=S) (unconditional so a duplicate ACK still
 *      gets answered even after A's own bookkeeping for S is gone).
 *   4. B receives ACK_ACK(seq=S): stops retrying the ACK, forgets S.
 *
 * Invariant: BNB_ACK_MAX_ACK_RETRIES must exceed BNB_ACK_MAX_SEND_RETRIES
 * (same retry cadence on both). This guarantees B is still holding its
 * dedup entry for S for at least as long as A could still be retransmitting
 * DATA(seq=S) — if that weren't true, B could give up on the ack-retry
 * before A gives up on the data-retry, drop its dedup entry, and then
 * re-deliver a later retransmit of DATA(seq=S) to the app a second time.
 *
 * All callbacks are invoked outside of bnb_ack's internal lock, using
 * stack-local copies of whatever data they need — they must not block for
 * long, and must not be assumed to run with the lock held.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "bnb_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// In-flight unacked reliable sends tracked per peer at once.
#define BNB_ACK_MAX_PENDING_SEND 4

// Acks sent but not yet ack-of-ack'd, tracked per peer at once. This also
// bounds the receive-side dedup window (how many distinct inbound seq_ids
// can be "already delivered, awaiting ack_ack" simultaneously).
#define BNB_ACK_MAX_PENDING_ACK 6

// Resend cadence for both pending-send and pending-ack table entries.
#define BNB_ACK_RETRY_INTERVAL_MS 300

// Poll period of the internal retry-sweep task.
#define BNB_ACK_SWEEP_INTERVAL_MS 100

// Retries before giving up on a reliable send and invoking failed_fn.
#define BNB_ACK_MAX_SEND_RETRIES 8

// Retries before giving up on resending an ACK. Must stay greater than
// BNB_ACK_MAX_SEND_RETRIES -- see the invariant documented above.
#define BNB_ACK_MAX_ACK_RETRIES 12

#if BNB_ACK_MAX_ACK_RETRIES <= BNB_ACK_MAX_SEND_RETRIES
#error "BNB_ACK_MAX_ACK_RETRIES must exceed BNB_ACK_MAX_SEND_RETRIES to avoid duplicate delivery"
#endif

// Wire payload carried by both BNB_MSG_ACK and BNB_MSG_ACK_ACK: the seq_id
// of the data message being acknowledged (an ACK never mints its own
// sequence number, it's identified entirely by the seq_id it's acking).
typedef struct __attribute__((packed)) {
    uint16_t seq_id;
} bnb_ack_payload_t;

/**
 * Called by bnb_ack whenever it needs to put bytes on the wire (initial
 * sends, retries, and its own ACK/ACK_ACK traffic). The implementation
 * should just call esp_now_send() and log on failure -- bnb_ack does not
 * touch ESP-NOW APIs itself.
 */
typedef void (*bnb_ack_transmit_fn_t)(const uint8_t peer_mac[6], const uint8_t *envelope,
                                       size_t envelope_len, void *ctx);

/**
 * Called exactly once per unique (peer, seq_id) data message, after it has
 * been fully received and dedup-checked, with the decoded payload.
 */
typedef void (*bnb_ack_delivered_fn_t)(const uint8_t peer_mac[6], bnb_msg_type_t type,
                                        const uint8_t *payload, size_t payload_len, void *ctx);

/**
 * Called when a reliable send hit BNB_ACK_MAX_SEND_RETRIES without ever
 * seeing an ACK. The message is dropped from the pending-send table before
 * this fires; there is no further retry.
 */
typedef void (*bnb_ack_failed_fn_t)(const uint8_t peer_mac[6], bnb_msg_type_t type,
                                     uint16_t seq_id, void *ctx);

/**
 * Initialize the bnb_ack module and start its internal retry-sweep task.
 * Must be called once before bnb_ack_send_reliable/bnb_ack_handle_incoming.
 *
 * @param max_peers maximum number of distinct peer MAC addresses to track
 *        (atlas should pass its shelf-count bound, shelf should pass 1)
 * @param transmit_fn required, called to actually put bytes on the wire
 * @param delivered_fn required, called once per unique inbound data message
 * @param failed_fn optional (may be NULL), called when a reliable send is
 *        given up on
 * @param ctx opaque pointer passed back to every callback invocation
 */
esp_err_t bnb_ack_init(size_t max_peers, bnb_ack_transmit_fn_t transmit_fn,
                        bnb_ack_delivered_fn_t delivered_fn, bnb_ack_failed_fn_t failed_fn, void *ctx);

/**
 * Queue a message for reliable delivery to peer_mac. Assigns the next
 * sequence ID for this peer, encodes it, and hands it to transmit_fn
 * immediately; the retry-sweep task takes over from there until an ACK is
 * seen or BNB_ACK_MAX_SEND_RETRIES is exhausted.
 *
 * @return ESP_OK if queued, ESP_ERR_NO_MEM if the peer or pending-send
 *         table is full, ESP_ERR_INVALID_SIZE if payload_len is too large,
 *         ESP_ERR_TIMEOUT if the internal lock could not be acquired
 */
esp_err_t bnb_ack_send_reliable(const uint8_t peer_mac[6], bnb_msg_type_t type,
                                 const void *payload, size_t payload_len);

/**
 * Feed a raw received ESP-NOW frame into bnb_ack. Decodes it, handles
 * BNB_MSG_ACK/BNB_MSG_ACK_ACK internally, and dispatches any new data
 * message to delivered_fn -- callers should call this from the task that
 * consumes their receive queue (not from the raw ESP-NOW receive
 * callback), same as they previously called bnb_protocol_decode there.
 */
void bnb_ack_handle_incoming(const uint8_t peer_mac[6], const uint8_t *raw_envelope, size_t raw_len);

#ifdef __cplusplus
}
#endif
