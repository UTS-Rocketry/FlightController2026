#include "telemetry.h"

void serial_print(const FlightSensorData *sensordata)
{
    if (sensordata == NULL) return;

    printf("Alt: %.2fm | H3LIS X:%.1f Y:%.1f Z:%.1f mg | "
           "IMU XL X:%.1f Y:%.1f Z:%.1f mg | "
           "GY X:%.1f Y:%.1f Z:%.1f mdps\r\n",

           sensordata->altitude,

           sensordata->x_mg,
           sensordata->y_mg,
           sensordata->z_mg,

           sensordata->x_mg_IMU,
           sensordata->y_mg_IMU,
           sensordata->z_mg_IMU,

           sensordata->x_gy,
           sensordata->y_gy,
           sensordata->z_gy);
}