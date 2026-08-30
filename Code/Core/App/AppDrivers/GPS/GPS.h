#ifndef ODIN_GPS_H
#define ODIN_GPS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	int32_t latitude_e7;
	int32_t longitude_e7;
	int32_t altitude_mm;
	uint32_t ground_speed_cms;
	uint32_t utc_ms;
	uint16_t course_cdeg;
	uint8_t satellites;
	uint8_t fix_quality;
	uint8_t status_flags;
	uint32_t last_update_ms;
} GPSFix;

enum {
	GPS_STATUS_FIX_VALID = (1U << 0),
	GPS_STATUS_GGA_SEEN = (1U << 1),
	GPS_STATUS_RMC_VALID = (1U << 2),
	GPS_STATUS_RX_OVERFLOW = (1U << 3),
};

int odin_gps_init(void);
bool odin_gps_get_fix(GPSFix *fix);
void odin_gps_thread(void *, void *, void *);

#endif
