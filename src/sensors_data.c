#include "sensors_data.h"

#include "common.h"
#include "shared_memory.h"
#include "motors.h"

sensors_data_t sensors_data = { 0 };

static const char* distance_file = "/home/pi/mmap_buffors/distance_buffor";
static const char* accel_file = "/home/pi/mmap_buffors/accelerometer_buffor";
static const char* gyro_mag_file = "/home/pi/mmap_buffors/sensors_buffor";
static const char* encoder_file = "/home/pi/mmap_buffors/encoder_buffor";

int sensors_data_init(void)
{
	syslog(LOG_INFO, "Initializing sensors subsystem...");
    memset((void*) &sensors_data, 0, sizeof(sensors_data_t));

	if (shared_memory_map_rdonly(distance_file, &sensors_data.dist_fd,
			(void*) &sensors_data.dist, sizeof(distance_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_ERR, "Cannot initialize sensors subsystem.");
		return -1;
	}

	if (shared_memory_map_rdonly(accel_file, &sensors_data.accel_fd,
			(void*) &sensors_data.accel, sizeof(accel_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_ERR, "Cannot initialize sensors subsystem.");
		return -1;
	}

	if (shared_memory_map_rdonly(gyro_mag_file, &sensors_data.gyro_mag_fd,
			(void*) &sensors_data.gyro_mag, sizeof(gyro_mag_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_ERR, "Cannot initialize sensors subsystem.");
		return -1;
	}

	if (shared_memory_map_rdwr(encoder_file, &sensors_data.encoder_fd,
			(void*) &sensors_data.encoder, sizeof(encoder_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_ERR, "Cannot initialize sensors subsystem.");
		return -1;
	}

	sensors_data_reset_odo();

	return 0;
}

void sensors_data_destroy(void)
{
	shared_memory_unmap(&sensors_data.dist_fd,
						 (void**) &sensors_data.dist, sizeof(distance_t));
	shared_memory_unmap(&sensors_data.accel_fd,
						 (void**) &sensors_data.accel, sizeof(accel_t));
	shared_memory_unmap(&sensors_data.gyro_mag_fd,
						 (void**) &sensors_data.gyro_mag, sizeof(gyro_mag_t));
	shared_memory_unmap(&sensors_data.encoder_fd,
						 (void**) &sensors_data.encoder, sizeof(encoder_t));
}

void sensors_data_filter(void)
{
	sensors_data.heading -= sensors_data.gyro_mag->gyro_yaw / 10;
	if (sensors_data.heading <= -180.0)
		sensors_data.heading = 360.0 + sensors_data.heading;
	else if (sensors_data.heading >= 180.0)
		sensors_data.heading = -360.0 + sensors_data.heading;

	sensors_data.odo = (sensors_data.encoder->left_dist +
		sensors_data.encoder->right_dist) * 100.0 / 2.0;
}

void sensors_data_reset_odo(void)
{
	sensors_data.encoder->clear = 1;
	sensors_data.odo = 0.0;
}
