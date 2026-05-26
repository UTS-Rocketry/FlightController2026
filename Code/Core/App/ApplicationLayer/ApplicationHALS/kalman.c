#include "kalman.h"
#include <string.h>


static KalmanFilter_t kf;

void kalman_init(void) { 

    kf.altitude = 0.0f;
    kf.accel_bias = 0.0f;
    kf.velocity = 0.0f;
    
    memset(kf.P, 0, sizeof(kf.P));

    kf.P[0] = 1.0f;
    kf.P[4] = 1.0f;
    kf.P[8] = 1.0f;

    kf.Q_altitude = 0.1f;
    kf.Q_velocity = 0.1f;
    kf.Q_bias = 0.01f;
    kf.R_altitude= 2.5f;

}
void kalman_predict(float accel_z_mg, float dt) {
    
    float accel_ms2 = (accel_z_mg / 1000.0f) * 9.81f;
    float accel_true = accel_ms2 - 9.81f - kf.accel_bias;

    kf.altitude = kf.altitude + kf.velocity * dt;
    kf.velocity = kf.velocity + accel_true * dt;
    kf.accel_bias = kf.accel_bias;

    kf.P[0] += kf.Q_altitude;
    kf.P[4] += kf.Q_velocity;
    kf.P[8] += kf.Q_bias;

}
void kalman_update(float baro_altitude) {
    float innovation = baro_altitude - kf.altitude;

    float S = kf.P[0] + kf.R_altitude;

    float K_altitude = kf.P[0] / S;
    float K_velocity = kf.P[3] / S;
    float K_bias     = kf.P[6] / S;
    
    kf.altitude   = kf.altitude   + K_altitude * innovation;
    kf.velocity   = kf.velocity   + K_velocity * innovation;
    kf.accel_bias = kf.accel_bias + K_bias     * innovation;

    kf.P[0] = (1.0f - K_altitude) * kf.P[0];
    kf.P[3] = (1.0f - K_altitude) * kf.P[3];
    kf.P[6] = (1.0f - K_altitude) * kf.P[6];


}

float kalman_get_altitude(void) {
    return kf.altitude;
}

float kalman_get_velocity(void) {
    return kf.velocity;
}