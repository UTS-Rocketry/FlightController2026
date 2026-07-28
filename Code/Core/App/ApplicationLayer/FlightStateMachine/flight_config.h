#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

/* ── Launch detect ─────────────────────────────────── */
#define LAUNCH_ACCEL_THRESHOLD_MG       3000.0f   /* 3g on IMU z axis */
#define LAUNCH_CONFIRM_SAMPLES          5          /* consecutive samples */

/* ── Burnout detect ────────────────────────────────── */
#define BURNOUT_ACCEL_THRESHOLD_MG      2000.0f   /* back to ~1g after motor out */
#define BURNOUT_CONFIRM_SAMPLES         5

/* ── Apogee detect ─────────────────────────────────── */
#define APOGEE_VELOCITY_THRESHOLD       -0.5f     /* m/s, negative = descending */
#define APOGEE_CONFIRM_SAMPLES          5

/* ── Deployment altitudes ──────────────────────────── */
#define MAIN_DEPLOY_ALT_M               300.0f    /* AGL meters */
#define MAIN_ALT_CONFIRM_SAMPLES        5

/* ── Landing detect ────────────────────────────────── */
#define LAND_VELOCITY_THRESHOLD         0.5f      /* m/s absolute */
#define LAND_ALT_THRESHOLD_M            10.0f     /* AGL meters */

/* ── State timeouts ────────────────────────────────── */
#define BOOST_TIMEOUT_MS                10000     /* 10s max burn */
#define COAST_TIMEOUT_MS                60000     /* 60s max coast */
#define DROGUE_TIMEOUT_MS               300000    /* 5 min max descent */
#define PARAFOIL_TIMEOUT_MS             300000    /* 5 min max descent */



#endif