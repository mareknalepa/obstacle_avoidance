#include "sensors_data.h"

#include <sys/mman.h>

sensors_data_t sensors_data = { 0 };

static const char* distance_file = "/home/pi/mmap_buffors/distance_buffor";
static const char* accel_file = "/home/pi/mmap_buffors/accelerometer_buffor";
static const char* gyro_mag_file = "/home/pi/mmap_buffors/sensors_buffor";
static const char* encoder_file = "/home/pi/mmap_buffors/encoder_buffor";

static int sensors_data_mmap_file(const char* file, int* fd, void** ptr,
								  int size)
{
    *fd = open(file, O_RDONLY | O_CLOEXEC);
    if (*fd < 0)
        return -1;
    
    *ptr = mmap(0, size, PROT_READ, MAP_SHARED, *fd, 0);
    if (*ptr == MAP_FAILED)
    {
        close(*fd);
        *fd = -1;
        return -1;
    }
    
    return 0;
}

static void sensors_data_unmap_file(int *fd, void** ptr, int size)
{
	if (*fd < 0)
		return;
	
	munmap(*ptr, size);
	*ptr = 0;
	close(*fd);
	*fd = -1;
}

int sensors_data_init()
{
	syslog(LOG_INFO, "Initializing sensors subsystem...");
    memset((void*) &sensors_data, 0, sizeof(sensors_data_t));
	
	if (sensors_data_mmap_file(distance_file, &sensors_data.dist_fd,
							 (void*) &sensors_data.dist,
							 sizeof(distance_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_DEBUG, "Cannot initialize sensors subsystem.");
		return -1;
	}
	
	if (sensors_data_mmap_file(accel_file, &sensors_data.accel_fd,
							 (void*) &sensors_data.accel,
							 sizeof(accel_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_DEBUG, "Cannot initialize sensors subsystem.");
		return -1;
	}
	
	if (sensors_data_mmap_file(gyro_mag_file, &sensors_data.gyro_mag_fd,
							 (void*) &sensors_data.gyro_mag,
							 sizeof(gyro_mag_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_DEBUG, "Cannot initialize sensors subsystem.");
		return -1;
	}
	
	if (sensors_data_mmap_file(encoder_file, &sensors_data.encoder_fd,
							 (void*) &sensors_data.encoder,
							 sizeof(encoder_t)) < 0)
	{
		sensors_data_destroy();
		syslog(LOG_DEBUG, "Cannot initialize sensors subsystem.");
		return -1;
	}
	
	return 0;
}

void sensors_data_destroy()
{
	sensors_data_unmap_file(&sensors_data.dist_fd,
						 (void*) &sensors_data.dist, sizeof(distance_t));
	sensors_data_unmap_file(&sensors_data.accel_fd,
						 (void*) &sensors_data.accel, sizeof(accel_t));
	sensors_data_unmap_file(&sensors_data.gyro_mag_fd,
						 (void*) &sensors_data.gyro_mag, sizeof(gyro_mag_t));
	sensors_data_unmap_file(&sensors_data.encoder_fd,
						 (void*) &sensors_data.encoder, sizeof(encoder_t));
}
