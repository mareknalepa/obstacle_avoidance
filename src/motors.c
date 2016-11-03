#include "motors.h"

#include "common.h"
#include <linux/i2c-dev.h>
#include "shared_memory.h"

static const char* motors_i2c_path = "/dev/i2c-1";
static const int motors_i2c_address = 0x47;
static int motors_i2c_fd = -1;

static const uint8_t motors_left_pwm = 4;
static const uint8_t motors_right_pwm = 5;
static const uint8_t motors_left_dir = 6;
static const uint8_t motors_right_dir= 7;
static const uint8_t motors_forward = 1;
static const uint8_t motors_backward = 0;

static const char* motors_file = "/home/pi/mmap_buffors/motors_buffor";
static int motors_fd = -1;

motors_t* motors = 0;

int motors_init(void)
{
	motors_i2c_fd = open(motors_i2c_path, O_RDWR | O_CLOEXEC);
	if (motors_i2c_fd < 0)
	{
		syslog(LOG_ERR, "Cannot initialize motors subsystem.");
		return -1;
	}

	if (ioctl(motors_i2c_fd, I2C_SLAVE, motors_i2c_address) < 0)
	{
		syslog(LOG_ERR, "Cannot initialize motors subsystem.");
		motors_destroy();
		return -1;
	}

	if (shared_memory_map_rdwr(motors_file, &motors_fd, (void*) &motors,
							 sizeof(motors_t)) < 0)
	{
		syslog(LOG_ERR, "Cannot initialize motors subsystem.");
		motors_destroy();
		return -1;
	}

	return 0;
}

void motors_destroy(void)
{
	close(motors_i2c_fd);
	motors_i2c_fd = -1;
	shared_memory_unmap(&motors_fd, (void**) &motors, sizeof(motors_t));
}

void motors_write(int left, int right)
{
	uint8_t byte;
	if (left < 0)
	{
		byte = (uint8_t) (left * -1);
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_left_dir,
								 motors_backward);
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_left_pwm, byte);
	}
	else
	{
		byte = (uint8_t) left;
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_left_dir,
								 motors_forward);
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_left_pwm, byte);
	}
	if (right < 0)
	{
		byte = (uint8_t) (right * -1);
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_right_dir,
								 motors_backward);
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_right_pwm, byte);
	}
	else
	{
		byte = (uint8_t) right;
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_right_dir,
								 motors_forward);
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_right_pwm, byte);
	}

	motors->left = left;
	motors->right = right;
}
