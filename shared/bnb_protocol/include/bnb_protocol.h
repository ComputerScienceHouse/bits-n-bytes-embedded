/**
 * File: bnb_protocol.h
 *
 * Purpose: Shared ESP-NOW wire envelope used by both atlas and shelf.
 * Wraps application payloads (currently JSON strings) with a type,
 * a reserved sequence ID for future QoS/ack use, and a CRC32 checksum
 * so corrupted or truncated packets can be detected and dropped instead
 * of misparsed.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum payload size carried inside an envelope. Header (9 bytes) +
// this must stay under ESP-NOW's 250-byte hard limit.
#define BNB_PROTOCOL_MAX_PAYLOAD_LEN 200

// Total on-the-wire size of an envelope header, in bytes.
#define BNB_PROTOCOL_HEADER_LEN 9

// Total maximum on-the-wire size of an encoded envelope, in bytes.
#define BNB_PROTOCOL_MAX_ENVELOPE_LEN (BNB_PROTOCOL_HEADER_LEN + BNB_PROTOCOL_MAX_PAYLOAD_LEN)

typedef enum {
    BNB_MSG_SHELF_STATUS = 1,   // shelf -> atlas: position (+ optional slot_id/delta_g)
    BNB_MSG_CALIBRATION  = 2,   // atlas -> shelf: slot_id + weight_g
    BNB_MSG_POSITION_SET = 3,   // atlas -> shelf: position swap request
} bnb_msg_type_t;

/**
 * Encode a payload into a checksummed envelope ready to hand to esp_now_send.
 *
 * @param type message type to tag the envelope with
 * @param payload payload bytes (e.g. a cJSON_PrintUnformatted string, without the NUL terminator)
 * @param payload_len number of payload bytes; must be <= BNB_PROTOCOL_MAX_PAYLOAD_LEN
 * @param out_buf destination buffer for the encoded envelope
 * @param out_buf_cap capacity of out_buf, in bytes
 * @param out_len set to the number of bytes written to out_buf on success
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null args, ESP_ERR_INVALID_SIZE if
 *         payload_len exceeds BNB_PROTOCOL_MAX_PAYLOAD_LEN or out_buf_cap is too small
 */
esp_err_t bnb_protocol_encode(bnb_msg_type_t type, const void *payload, size_t payload_len,
                               uint8_t *out_buf, size_t out_buf_cap, size_t *out_len);

/**
 * Decode and checksum-validate a received envelope.
 *
 * @param data raw bytes received over ESP-NOW
 * @param len number of bytes in data
 * @param out_type set to the envelope's message type on success
 * @param out_payload destination buffer for the decoded payload bytes
 * @param out_payload_cap capacity of out_payload, in bytes
 * @param out_payload_len set to the number of payload bytes written on success
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG on null args, ESP_ERR_INVALID_SIZE if
 *         the declared payload length doesn't fit the received data or out_payload_cap,
 *         ESP_ERR_INVALID_CRC if the checksum does not match
 */
esp_err_t bnb_protocol_decode(const uint8_t *data, size_t len, bnb_msg_type_t *out_type,
                               uint8_t *out_payload, size_t out_payload_cap, size_t *out_payload_len);

#ifdef __cplusplus
}
#endif
