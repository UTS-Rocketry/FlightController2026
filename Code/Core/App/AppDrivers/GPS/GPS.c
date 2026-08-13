#include "GPS.h"
#include "main.h"
#include <stddef.h>
#include <string.h>

#define GPS_RX_RING_SIZE 256U
#define GPS_SENTENCE_SIZE 128U
#define GPS_MAX_FIELDS 20U

static UART_HandleTypeDef *gps_uart;
static uint8_t irq_byte;
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_overflow;
static uint8_t rx_ring[GPS_RX_RING_SIZE];
static char sentence[GPS_SENTENCE_SIZE];
static uint16_t sentence_length;
static GPSFix latest_fix;
static bool have_nmea_data;

static void parse_sentence(char *line);
static bool checksum_valid(const char *line);
static uint32_t parse_unsigned(const char *text);
static int32_t parse_fixed(const char *text, uint32_t scale);
static bool parse_coordinate(const char *text, char hemisphere, int32_t *degrees_e7);
static uint32_t parse_utc_ms(const char *text);

HAL_StatusTypeDef GPS_init(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart->Instance != UART5) return HAL_ERROR;

    gps_uart = uart;
    rx_head = 0U;
    rx_tail = 0U;
    sentence_length = 0U;
    have_nmea_data = false;
    rx_overflow = false;
    memset(&latest_fix, 0, sizeof(latest_fix));

    /* GPS2Reset is the reset line paired with UART5. The reset input on the
     * intended GPS board is active-low. */
    HAL_GPIO_WritePin(GPS2ResetPin_GPIO_Port, GPS2ResetPin_Pin, GPIO_PIN_RESET);
    HAL_Delay(10U);
    HAL_GPIO_WritePin(GPS2ResetPin_GPIO_Port, GPS2ResetPin_Pin, GPIO_PIN_SET);
    HAL_Delay(10U);

    return HAL_UART_Receive_IT(gps_uart, &irq_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (gps_uart == NULL || uart != gps_uart) return;

    uint16_t next = (uint16_t)((rx_head + 1U) % GPS_RX_RING_SIZE);
    if (next != rx_tail) {
        rx_ring[rx_head] = irq_byte;
        rx_head = next;
    } else {
        rx_overflow = true;
    }

    (void)HAL_UART_Receive_IT(gps_uart, &irq_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (gps_uart == NULL || uart != gps_uart) return;

    rx_overflow = true;
    (void)HAL_UART_Receive_IT(gps_uart, &irq_byte, 1U);
}

void GPS_service(void)
{
    while (rx_tail != rx_head) {
        char byte = (char)rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % GPS_RX_RING_SIZE);

        if (byte == '$') {
            sentence_length = 0U;
            sentence[sentence_length++] = byte;
        } else if (byte == '\n') {
            if (sentence_length > 0U) {
                sentence[sentence_length] = '\0';
                parse_sentence(sentence);
            }
            sentence_length = 0U;
        } else if (byte != '\r' && sentence_length > 0U) {
            if (sentence_length < (GPS_SENTENCE_SIZE - 1U)) {
                sentence[sentence_length++] = byte;
            } else {
                sentence_length = 0U;
            }
        }
    }
}

bool GPS_get_fix(GPSFix *fix)
{
    if (fix == NULL || !have_nmea_data) return false;
    *fix = latest_fix;
    if (rx_overflow) fix->status_flags |= GPS_STATUS_RX_OVERFLOW;
    return true;
}

static void parse_sentence(char *line)
{
    if (!checksum_valid(line)) return;

    char *asterisk = strchr(line, '*');
    if (asterisk == NULL) return;
    *asterisk = '\0';

    char *fields[GPS_MAX_FIELDS] = {0};
    uint8_t count = 0U;
    char *field = line;
    while (field != NULL && count < GPS_MAX_FIELDS) {
        fields[count++] = field;
        char *comma = strchr(field, ',');
        if (comma == NULL) break;
        *comma = '\0';
        field = comma + 1;
    }

    if (count == 0U) return;

    bool is_gga = strcmp(fields[0], "$GPGGA") == 0 || strcmp(fields[0], "$GNGGA") == 0;
    bool is_rmc = strcmp(fields[0], "$GPRMC") == 0 || strcmp(fields[0], "$GNRMC") == 0;

    if (is_gga && count > 9U) {
        uint8_t quality = (uint8_t)parse_unsigned(fields[6]);
        int32_t latitude;
        int32_t longitude;

        latest_fix.utc_ms = parse_utc_ms(fields[1]);
        latest_fix.fix_quality = quality;
        latest_fix.satellites = (uint8_t)parse_unsigned(fields[7]);
        latest_fix.altitude_mm = parse_fixed(fields[9], 1000U);
        latest_fix.status_flags |= GPS_STATUS_GGA_SEEN;

        if (quality > 0U && parse_coordinate(fields[2], fields[3][0], &latitude) &&
            parse_coordinate(fields[4], fields[5][0], &longitude)) {
            latest_fix.latitude_e7 = latitude;
            latest_fix.longitude_e7 = longitude;
            latest_fix.status_flags |= GPS_STATUS_FIX_VALID;
        } else {
            latest_fix.status_flags &= (uint8_t)~GPS_STATUS_FIX_VALID;
        }

        latest_fix.last_update_ms = HAL_GetTick();
        have_nmea_data = true;
    } else if (is_rmc && count > 8U) {
        bool valid = fields[2][0] == 'A';
        int32_t latitude;
        int32_t longitude;

        latest_fix.utc_ms = parse_utc_ms(fields[1]);
        latest_fix.ground_speed_cms =
            (uint32_t)(((uint64_t)(uint32_t)parse_fixed(fields[7], 1000U) * 51444U + 500000U) /
                       1000000U);
        latest_fix.course_cdeg = (uint16_t)parse_fixed(fields[8], 100U);

        if (valid && parse_coordinate(fields[3], fields[4][0], &latitude) &&
            parse_coordinate(fields[5], fields[6][0], &longitude)) {
            latest_fix.latitude_e7 = latitude;
            latest_fix.longitude_e7 = longitude;
            latest_fix.status_flags |= GPS_STATUS_RMC_VALID;
            latest_fix.status_flags |= GPS_STATUS_FIX_VALID;
        } else {
            latest_fix.status_flags &= (uint8_t)~GPS_STATUS_RMC_VALID;
            if (latest_fix.fix_quality == 0U) {
                latest_fix.status_flags &= (uint8_t)~GPS_STATUS_FIX_VALID;
            }
        }

        latest_fix.last_update_ms = HAL_GetTick();
        have_nmea_data = true;
    }
}

static bool checksum_valid(const char *line)
{
    if (line == NULL || line[0] != '$') return false;

    const char *asterisk = strchr(line, '*');
    if (asterisk == NULL || asterisk[1] == '\0' || asterisk[2] == '\0') return false;

    uint8_t checksum = 0U;
    for (const char *p = line + 1; p < asterisk; ++p) checksum ^= (uint8_t)*p;

    uint8_t received = 0U;
    for (uint8_t i = 1U; i <= 2U; ++i) {
        char c = asterisk[i];
        uint8_t nibble;
        if (c >= '0' && c <= '9') nibble = (uint8_t)(c - '0');
        else if (c >= 'A' && c <= 'F') nibble = (uint8_t)(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') nibble = (uint8_t)(c - 'a' + 10);
        else return false;
        received = (uint8_t)((received << 4) | nibble);
    }

    return checksum == received;
}

static uint32_t parse_unsigned(const char *text)
{
    uint32_t value = 0U;
    if (text == NULL) return 0U;
    while (*text >= '0' && *text <= '9') {
        value = value * 10U + (uint32_t)(*text - '0');
        ++text;
    }
    return value;
}

static int32_t parse_fixed(const char *text, uint32_t scale)
{
    if (text == NULL || *text == '\0') return 0;

    bool negative = false;
    if (*text == '-') {
        negative = true;
        ++text;
    } else if (*text == '+') {
        ++text;
    }

    uint32_t whole = parse_unsigned(text);
    while (*text >= '0' && *text <= '9') ++text;

    uint32_t fraction = 0U;
    uint32_t divisor = 1U;
    if (*text == '.') {
        ++text;
        while (*text >= '0' && *text <= '9' && divisor < scale) {
            fraction = fraction * 10U + (uint32_t)(*text - '0');
            divisor *= 10U;
            ++text;
        }
    }

    int64_t value = (int64_t)whole * scale;
    if (divisor > 1U) value += ((int64_t)fraction * scale + divisor / 2U) / divisor;
    return negative ? (int32_t)-value : (int32_t)value;
}

static bool parse_coordinate(const char *text, char hemisphere, int32_t *degrees_e7)
{
    if (text == NULL || *text == '\0' || degrees_e7 == NULL) return false;
    if (hemisphere != 'N' && hemisphere != 'S' && hemisphere != 'E' && hemisphere != 'W') {
        return false;
    }

    const char *decimal = strchr(text, '.');
    size_t whole_digits = decimal == NULL ? strlen(text) : (size_t)(decimal - text);
    if (whole_digits < 3U) return false;

    size_t degree_digits = whole_digits - 2U;
    uint32_t degrees = 0U;
    for (size_t i = 0U; i < degree_digits; ++i) {
        if (text[i] < '0' || text[i] > '9') return false;
        degrees = degrees * 10U + (uint32_t)(text[i] - '0');
    }

    int32_t minutes_e6 = parse_fixed(text + degree_digits, 1000000U);
    int64_t result = (int64_t)degrees * 10000000LL +
                     ((int64_t)minutes_e6 * 10LL + 30LL) / 60LL;
    if (hemisphere == 'S' || hemisphere == 'W') result = -result;
    *degrees_e7 = (int32_t)result;
    return true;
}

static uint32_t parse_utc_ms(const char *text)
{
    if (text == NULL || strlen(text) < 6U) return 0U;
    for (uint8_t i = 0U; i < 6U; ++i) {
        if (text[i] < '0' || text[i] > '9') return 0U;
    }

    uint32_t hours = (uint32_t)(text[0] - '0') * 10U + (uint32_t)(text[1] - '0');
    uint32_t minutes = (uint32_t)(text[2] - '0') * 10U + (uint32_t)(text[3] - '0');
    uint32_t seconds = (uint32_t)(text[4] - '0') * 10U + (uint32_t)(text[5] - '0');
    uint32_t milliseconds = 0U;

    if (text[6] == '.') {
        uint32_t place = 100U;
        for (uint8_t i = 7U; text[i] >= '0' && text[i] <= '9' && place > 0U; ++i) {
            milliseconds += (uint32_t)(text[i] - '0') * place;
            place /= 10U;
        }
    }

    if (hours > 23U || minutes > 59U || seconds > 60U) return 0U;
    return ((hours * 60U + minutes) * 60U + seconds) * 1000U + milliseconds;
}
