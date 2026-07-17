#include "bnb_protocol.h"

#include <string.h>
#include "esp_rom_crc.h"

// Wire layout (little-endian, packed):
//   offset 0: uint8_t  msg_type
//   offset 1: uint16_t seq_id
//   offset 3: uint16_t payload_len
//   offset 5: uint32_t crc32        (CRC32 over payload bytes only)
//   offset 9: payload bytes

static void put_u16(uint8_t *buf, uint16_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

static uint16_t get_u16(const uint8_t *buf) {
    return (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
}

static void put_u32(uint8_t *buf, uint32_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

static uint32_t get_u32(const uint8_t *buf) {
    return (uint32_t)buf[0]
           | ((uint32_t)buf[1] << 8)
           | ((uint32_t)buf[2] << 16)
           | ((uint32_t)buf[3] << 24);
}

esp_err_t bnb_protocol_encode(bnb_msg_type_t type, uint16_t seq_id, const void *payload, size_t payload_len,
                               uint8_t *out_buf, size_t out_buf_cap, size_t *out_len) {
    if (out_buf == NULL || out_len == NULL || (payload == NULL && payload_len > 0)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (payload_len > BNB_PROTOCOL_MAX_PAYLOAD_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }
    const size_t envelope_len = BNB_PROTOCOL_HEADER_LEN + payload_len;
    if (out_buf_cap < envelope_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint32_t crc = payload_len > 0 ? esp_rom_crc32_le(0, (const uint8_t *)payload, payload_len) : 0;

    out_buf[0] = (uint8_t)type;
    put_u16(&out_buf[1], seq_id);
    put_u16(&out_buf[3], (uint16_t)payload_len);
    put_u32(&out_buf[5], crc);
    if (payload_len > 0) {
        memcpy(&out_buf[BNB_PROTOCOL_HEADER_LEN], payload, payload_len);
    }

    *out_len = envelope_len;
    return ESP_OK;
}

esp_err_t bnb_protocol_decode(const uint8_t *data, size_t len, bnb_msg_type_t *out_type, uint16_t *out_seq_id,
                               uint8_t *out_payload, size_t out_payload_cap, size_t *out_payload_len) {
    if (data == NULL || out_type == NULL || out_seq_id == NULL || out_payload == NULL || out_payload_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (len < BNB_PROTOCOL_HEADER_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint16_t payload_len = get_u16(&data[3]);
    const uint32_t expected_crc = get_u32(&data[5]);

    if (payload_len > BNB_PROTOCOL_MAX_PAYLOAD_LEN
        || (size_t)(BNB_PROTOCOL_HEADER_LEN + payload_len) > len
        || payload_len > out_payload_cap) {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint8_t *payload = &data[BNB_PROTOCOL_HEADER_LEN];
    const uint32_t actual_crc = payload_len > 0 ? esp_rom_crc32_le(0, payload, payload_len) : 0;
    if (actual_crc != expected_crc) {
        return ESP_ERR_INVALID_CRC;
    }

    *out_type = (bnb_msg_type_t)data[0];
    *out_seq_id = get_u16(&data[1]);
    if (payload_len > 0) {
        memcpy(out_payload, payload, payload_len);
    }
    *out_payload_len = payload_len;
    return ESP_OK;
}
