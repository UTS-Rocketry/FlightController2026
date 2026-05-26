#ifndef KALMAN_H
#define KALMAN_H


typedef struct{

    float altitude;
    float velocity;
    float accel_bias;

    float P[9];

    float Q_altitude;
    float Q_velocity;
    float Q_bias;
    float R_altitude;

}KalmanFilter_t;

void kalman_init(void);
void kalman_predict(float accel_z_mg, float dt);
void kalman_update(float baro_altitude);
float kalman_get_altitude(void);
float kalman_get_velocity(void);

#endif