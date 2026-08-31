#include "kalman.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

static KalmanFilter_t kf;

void kalman_init(void) {
    kf.altitude   = 0.0f;
    kf.velocity   = 0.0f;
    kf.accel_bias = 0.0f;

    memset(kf.P, 0, sizeof(kf.P));
    kf.P[0] = 1.0f;
    kf.P[4] = 1.0f;
    kf.P[8] = 1.0f;

    kf.Q_altitude = 0.1f;
    kf.Q_velocity = 0.1f;
    kf.Q_bias     = 0.01f;
    kf.R_altitude = 2.5f; /* static noise var measured at 0.70 (std 0.835m); inflated */
}

void kalman_predict(float accel_z_mg, float dt, bool freeze_bias) {
    
    // Sanity check inputs — reject garbage on first call
    if (isnan(accel_z_mg) || isinf(accel_z_mg)) return;
    if (dt <= 0.0f || dt > 1.0f) return;

    // z_mg_IMU reads ~1000mg at rest (1g), subtract gravity
    float accel_ms2  = (accel_z_mg / 1000.0f) * 9.81f;
    // float accel_true = accel_ms2 - 9.81f;
    float accel_true = accel_ms2 - 9.81f - kf.accel_bias;

    // Clamp accel to ±160 m/s² (±16g) — rejects any residual garbage
    if (accel_true >  160.0f) accel_true =  160.0f;
    if (accel_true < -160.0f) accel_true = -160.0f;

    // State predict
    kf.altitude += kf.velocity * dt;
    kf.velocity += accel_true * dt;

    // Covariance predict: P = F*P*Fᵀ + Q
    // F = | 1  dt   0  |
    //     | 0   1  -dt |
    //     | 0   0   1  |
    float p[9];
    memcpy(p, kf.P, sizeof(p));

    // Step 1: FP = F * P
    float fp00 = p[0] + dt*p[3];
    float fp01 = p[1] + dt*p[4];
    float fp02 = p[2] + dt*p[5];
    float fp10 = p[3] - dt*p[6];
    float fp11 = p[4] - dt*p[7];
    float fp12 = p[5] - dt*p[8];
    float fp20 = p[6];
    float fp21 = p[7];
    float fp22 = p[8];

    // Step 2: P = FP * Fᵀ
    kf.P[0] = fp00 + fp01*dt;
    kf.P[1] = fp01 - fp02*dt;
    kf.P[2] = fp02;
    kf.P[3] = fp10 + fp11*dt;
    kf.P[4] = fp11 - fp12*dt;
    kf.P[5] = fp12;
    kf.P[6] = fp20 + fp21*dt;
    kf.P[7] = fp21 - fp22*dt;
    kf.P[8] = fp22;

    // Step 3: Add process noise, scaled relative to the 10ms nominal tick
    // this Q was tuned for. A stalled loop (larger dt) now correctly
    // injects proportionally more uncertainty instead of a flat amount.
    float dt_scale = dt / 0.01f;
    kf.P[0] += kf.Q_altitude * dt_scale;
    kf.P[4] += kf.Q_velocity * dt_scale;
    // If bias correction is frozen (update() will force K2=0), its
    // covariance must stop growing too. Left unbounded, P[8] grows for
    // the rest of the flight with nothing to correct it, and leaks into
    // velocity covariance through the fp10/fp11/fp12 cross-coupling above
    // — confirmed on real flight data to eventually destabilize P4
    // (velocity variance goes negative). Halting K2 alone is not enough.
    if (!freeze_bias) {
        kf.P[8] += kf.Q_bias * dt_scale;
    }
}

void kalman_update(float baro_altitude, bool freeze_bias) {
    if (isnan(baro_altitude) || isinf(baro_altitude)) return;

    float y     = baro_altitude - kf.altitude;
    float S     = kf.P[0] + kf.R_altitude;

    // Innovation gate: reject a barometer sample that disagrees with the
    // prediction by more than 5 standard deviations. Protects against a
    // single bad/glitched reading (e.g. static-port disturbance during
    // coning) yanking the whole state. Tune sigma once validated offline.
    float innovation_std = sqrtf(S);
    if (fabsf(y) > 5.0f * innovation_std) {
        return;
    }

    float S_inv = 1.0f / S;

    float K0 = kf.P[0] * S_inv;
    float K1 = kf.P[3] * S_inv;
    // Bias correction is frozen once armed/at boost, so barometer lag and
    // sensor disagreement can't be silently absorbed into the bias term
    // for the rest of the flight (see kalman.md notes / review).
    float K2 = freeze_bias ? 0.0f : kf.P[6] * S_inv;

    kf.altitude   += K0 * y;
    kf.velocity   += K1 * y;
    kf.accel_bias += K2 * y;

    float p[9];
    memcpy(p, kf.P, sizeof(p));

    kf.P[0] = p[0] - K0*p[0];
    kf.P[1] = p[1] - K0*p[1];
    kf.P[2] = p[2] - K0*p[2];
    kf.P[3] = p[3] - K1*p[0];
    kf.P[4] = p[4] - K1*p[1];
    kf.P[5] = p[5] - K1*p[2];
    kf.P[6] = p[6] - K2*p[0];
    kf.P[7] = p[7] - K2*p[1];
    kf.P[8] = p[8] - K2*p[2];

    // Force exact symmetry (P[1]==P[3], P[2]==P[6], P[5]==P[7]). A
    // covariance matrix is symmetric by construction; floating-point
    // roundoff accumulates over enough iterations (thousands of records
    // across a long flight + ground-sitting time) and can silently break
    // that, eventually costing the matrix its positive-definiteness and
    // diverging the whole filter. Confirmed on real flight data — this
    // is the fix that actually resolved it, not just the freeze_bias
    // changes above on their own. Cheap, standard, do this every update.
    kf.P[1] = kf.P[3] = 0.5f * (kf.P[1] + kf.P[3]);
    kf.P[2] = kf.P[6] = 0.5f * (kf.P[2] + kf.P[6]);
    kf.P[5] = kf.P[7] = 0.5f * (kf.P[5] + kf.P[7]);
}

float kalman_get_altitude(void) { return kf.altitude; }
float kalman_get_velocity(void) { return kf.velocity; }