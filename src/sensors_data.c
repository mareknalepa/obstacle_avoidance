#include "sensors_data.h"

sensors_data_t sensors_data = { 0 };

static const char* distance_file = "/home/pi/distance_buffor";
static const char* accel_file = "/home/pi/accelerometer_buffor";
static const char* gyro_mag_file = "/home/pi/sensors_buffor";

int sensors_data_init()
{
	syslog(LOG_INFO, "Initializing sensors subsystem...");

	sensors_data.distance_file = fopen(distance_file, "rb");
	if (!sensors_data.distance_file)
	{
		syslog(LOG_ERR, "Cannot initialize sensors subsystem.");
		return -1;
	}

	sensors_data.accel_file = fopen(accel_file, "rb");
	if (!sensors_data.accel_file)
	{
		syslog(LOG_ERR, "Cannot initialize sensors subsystem.");
		return -1;
	}

	sensors_data.gyro_mag_file = fopen(gyro_mag_file, "rb");
	if (!sensors_data.gyro_mag_file)
	{
		syslog(LOG_ERR, "Cannot initialize sensors subsystem.");
		return -1;
	}

	return 0;
}

void sensors_data_destroy()
{
	fclose(sensors_data.distance_file);
	fclose(sensors_data.accel_file);
	fclose(sensors_data.gyro_mag_file);
}

int sensors_data_read()
{
	rewind(sensors_data.distance_file);
	if (fread((void*) &sensors_data.dist, sizeof(distance_t), 1,
			sensors_data.distance_file) != sizeof(distance_t))
	{
		syslog(LOG_ERR, "Error while reading sensors data.");
		return -1;
	}

	rewind(sensors_data.accel_file);
	if (fread((void*) &sensors_data.accel, sizeof(accel_t), 1,
			sensors_data.accel_file) != sizeof(accel_t))
	{
		syslog(LOG_ERR, "Error while reading sensors data.");
		return -1;
	}

	rewind(sensors_data.gyro_mag_file);
	if (fread((void*) &sensors_data.gyro_mag, sizeof(gyro_mag_t), 1,
			sensors_data.gyro_mag_file) != sizeof(gyro_mag_t))
	{
		syslog(LOG_ERR, "Error while reading sensors data.");
		return -1;
	}

	return 0;
}
