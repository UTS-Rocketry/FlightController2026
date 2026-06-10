#include "crc16.h"

#define POLY 0x1021
uint16_t crc16(uint16_t crc, const void *in_data, uint64_t len) {
    const uint8_t *data = in_data;
    for (uint64_t i = 0; i < len; i++) {
        crc = crc ^ (data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ POLY;
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}