#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* UART5 is routed to the GPS JST connector. The GPS must output NMEA 0183
 * GGA and/or RMC sentences at the baud rate configured for UART5. */
typedef struct {
    int32_t latitude_e7;       /* signed decimal degrees * 10^7 */
    int32_t longitude_e7;      /* signed decimal degrees * 10^7 */
    int32_t altitude_mm;       /* altitude above mean sea level */
    uint32_t ground_speed_cms; /* centimetres per second */
    uint32_t utc_ms;           /* milliseconds since 00:00:00 UTC */
    uint16_t course_cdeg;      /* course over ground, degrees * 100 */
    uint8_t satellites;
    uint8_t fix_quality;       /* NMEA GGA quality; 0 means no fix */
    uint8_t status_flags;
    uint32_t last_update_ms;   /* local HAL tick of last valid NMEA sentence */
} GPSFix;

enum {
    GPS_STATUS_FIX_VALID   = (1U << 0),
    GPS_STATUS_GGA_SEEN    = (1U << 1),
    GPS_STATUS_RMC_VALID   = (1U << 2),
    GPS_STATUS_RX_OVERFLOW = (1U << 3)
};

HAL_StatusTypeDef GPS_init(UART_HandleTypeDef *uart);
void GPS_service(void);
bool GPS_get_fix(GPSFix *fix);

#ifdef __cplusplus
}
#endif

#endif
