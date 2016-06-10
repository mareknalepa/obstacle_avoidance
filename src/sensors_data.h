#ifndef SENSORS_DATA_H_
#define SENSORS_DATA_H_

#include "common.h"

typedef struct {
    int distance;
} distance_t;

typedef struct {
    float m2_x;
    float m2_y;
    float m2_z;
    float g_x;
    float g_y;
    float g_z;
} accel_t;

typedef struct {
    float gyro_yaw;
    float gyro_pitch;
    float gyro_roll;
    float mag_x;
    float mag_y;
    float mag_z;
} gyro_mag_t;

typedef struct {
    int encoder_pid;
    float right_dist;
    float right_speed;
    float left_dist;
    float left_speed;
} encoder_t;

typedef struct {
    int dist_fd;
    distance_t* dist;
    int accel_fd;
    accel_t* accel;
    int gyro_mag_fd;
    gyro_mag_t* gyro_mag;
    int encoder_fd;
    encoder_t* encoder;
} sensors_data_t;

extern sensors_data_t sensors_data;

int sensors_data_init();
void sensors_data_destroy();

#endif /* SENSORS_DATA_H_ */
