#include "watchdog.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>


#define RESET_BKP_MAGIC   0x5A00u            /* validity tag in upper 16 bits */
#define RESET_BKP_MASK    0xFFFF0000u

void reset_cause_check(void)
{
    /* PWR clock already enabled in SystemClock_Config(); lift backup-domain
       write protection so we can touch RTC->BKPxR. */
    HAL_PWR_EnableBkUpAccess();

    /* Initialize the persistent counter if the magic tag isn't present
       (first-ever boot or uninitialized backup domain). */
    uint32_t bkp = RTC->BKP2R;
    if ((bkp & RESET_BKP_MASK) != ((uint32_t)RESET_BKP_MAGIC << 16)) {
        RTC->BKP2R = ((uint32_t)RESET_BKP_MAGIC << 16);   /* tag set, count 0 */
        bkp = RTC->BKP2R;
    }
    uint16_t iwdg_count = (uint16_t)(bkp & 0xFFFF);

    /* Decode reset cause. Multiple flags can be set; check watchdog first,
       PINRST last since it's set on almost every reset and would mask others. */
    const char *cause = "OTHER/UNKNOWN";

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        cause = "IWDG (watchdog)";
        iwdg_count++;
        RTC->BKP2R = ((uint32_t)RESET_BKP_MAGIC << 16) | iwdg_count;
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
        cause = "WWDG (window watchdog)";
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
        cause = "SOFTWARE";
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
        cause = "BROWNOUT (BOR)";
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
        cause = "POWER-ON (POR/PDR)";
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
        cause = "PIN (NRST)";
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
        cause = "LOW-POWER";
    }

#ifdef DEBUG
    printf("RESET CAUSE: %s | total IWDG resets: %u\r\n", cause, iwdg_count);
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        printf("!!! WATCHDOG RESET -- investigate main-loop blocking !!!\r\n");
    }
#else
    (void)cause;
#endif

    /* Clear all reset flags so the next boot reports its own cause cleanly.
       Must come AFTER reading every flag above. */
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

uint16_t reset_get_iwdg_count(void)
{
    uint32_t bkp = RTC->BKP2R;
    if ((bkp & RESET_BKP_MASK) != ((uint32_t)RESET_BKP_MAGIC << 16)) {
        return 0;   /* not yet initialized */
    }
    return (uint16_t)(bkp & 0xFFFF);
}