/*
 * sensors_data.c
 *
 * Author: Marek Nalepa
 * Purpose: Provide fresh sensors and derived data.
 */

#include "sensors_data.h"

#include "common.h"
#include "constants.h"
#include "shared_memory.h"
#include "motors.h"
#include <math.h>

sensors_data_t sensors_data = { 0 };

/* Structs for obtaining sensors data */
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
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;
    float mag_x;
    float mag_y;
    float mag_z;
} gyro_mag_t;

typedef struct {
    int encoder_pid;
    int clear;
    float right_dist;
    float right_speed;
    float left_dist;
    float left_speed;
} encoder_t;

/* Memory mapping paths */
static const char* distance_file = "/home/pi/mmap_buffors/distance_buffor";
static const char* accel_file = "/home/pi/mmap_buffors/accelerometer_buffor";
static const char* gyro_mag_file = "/home/pi/mmap_buffors/sensors_buffor";
static const char* encoder_file = "/home/pi/mmap_buffors/encoder_buffor";

/* Raw sensors data */
static int dist_fd;
static distance_t* dist;
static int accel_fd;
static accel_t* accel;
static int gyro_mag_fd;
static gyro_mag_t* gyro_mag;
static int encoder_fd;
static encoder_t* encoder;

/* Internal variables */
static double encoder_left_prev;
static double encoder_right_prev;
static double heading_sin;
static double heading_cos;
static double raw_odo;

/* Init sensors subsytem */
int sensors_data_init(void)
{
	syslog(LOG_INFO, "Initializing sensors subsystem...");
    memset((void*) &sensors_data, 0, sizeof(sensors_data_t));

	/* Map memory so it is shared with main daemon */
	if (shared_memory_map_rdonly(distance_file, &dist_fd,
			(void*) &dist, sizeof(distance_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_ERR, "Cannot initialize sensors subsystem.");
		return -1;
	}

	if (shared_memory_map_rdonly(accel_file, &accel_fd,
			(void*) &accel, sizeof(accel_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_ERR, "Cannot initialize sensors subsystem.");
		return -1;
	}

	if (shared_memory_map_rdonly(gyro_mag_file, &gyro_mag_fd,
			(void*) &gyro_mag, sizeof(gyro_mag_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_ERR, "Cannot initialize sensors subsystem.");
		return -1;
	}

	if (shared_memory_map_rdwr(encoder_file, &encoder_fd,
			(void*) &encoder, sizeof(encoder_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_ERR, "Cannot initialize sensors subsystem.");
		return -1;
	}

	encoder_left_prev = encoder->left_dist;
	encoder_right_prev = encoder->right_dist;
	sensors_data.odo = 0.0;
	raw_odo = 0.0;
	sensors_data_reset_coordinates();

	return 0;
}

/* Shutdown sensors system */
void sensors_data_destroy(void)
{
	shared_memory_unmap(&dist_fd, (void**) &dist, sizeof(distance_t));
	shared_memory_unmap(&accel_fd, (void**) &accel, sizeof(accel_t));
	shared_memory_unmap(&gyro_mag_fd, (void**) &gyro_mag, sizeof(gyro_mag_t));
	shared_memory_unmap(&encoder_fd, (void**) &encoder, sizeof(encoder_t));
}

/* Filter data and calculate derived values */
void sensors_data_filter(void)
{
	/* Filter out invalid distances from ultrasonic sensor */
	if (dist->distance > 32000 || dist->distance < 5)
		sensors_data.distance = 0.0;
	else
		sensors_data.distance = dist->distance;

	/* Calculate current heading by integrating gyroscope data */
	double prev_heading = sensors_data.heading;
	sensors_data.heading -= gyro_mag->gyro_yaw / 10;
	sensors_data.heading_rate = sensors_data.heading - prev_heading;
	if (sensors_data.heading <= -180.0)
		sensors_data.heading = 360.0 + sensors_data.heading;
	else if (sensors_data.heading >= 180.0)
		sensors_data.heading = -360.0 + sensors_data.heading;

	/* Calculate increase in odometer */
	double left_d = encoder->left_dist - encoder_left_prev;
	double right_d = encoder->right_dist - encoder_right_prev;
	if (motors->left < 0)
		left_d *= -1.0;
	if (motors->right < 0)
		right_d *= -1.0;
	double odo_d = (left_d + right_d) * 100.0 / 2.0;
	raw_odo += odo_d;

	/* Apply exponential smoothing to odometer */
	double prev_odo = sensors_data.odo;
	sensors_data.odo += ODO_FILTER_SMOOTHNESS * (raw_odo - sensors_data.odo);
	encoder_left_prev = encoder->left_dist;
	encoder_right_prev = encoder->right_dist;

	/* Update current position in XY coordinates system */
	if (sensors_data.heading_rate != 0)
	{
		heading_sin = sin(sensors_data.heading * M_PI / 180.0);
		heading_cos = cos(sensors_data.heading * M_PI / 180.0);
	}
	if (motors->left * motors->right >= 0)
	{
		sensors_data.position_x += heading_sin * (sensors_data.odo - prev_odo);
		sensors_data.position_y += heading_cos * (sensors_data.odo - prev_odo);
	}
}

/* Reset coordinates system to begin in current position */
void sensors_data_reset_coordinates(void)
{
	sensors_data.heading = 0.0;
	sensors_data.position_x = 0.0;
	sensors_data.position_y = 0.0;
	heading_sin = 0.0;
	heading_cos = 1.0;
}
