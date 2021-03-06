/*
 * motors.c
 *
 * Author: Marek Nalepa, ported from C++ to C, original code by Wojciech Michna
 * Purpose: Control vehicle's motors.
 */

#include "motors.h"

#include "common.h"
#include <linux/i2c-dev.h>
#include "shared_memory.h"

static const char* motors_i2c_path = "/dev/i2c-1";
static const int motors_i2c_address = 0x47;
static int motors_i2c_fd = -1;

/* Romeo V2 registers' addresses */
static const uint8_t motors_left_pwm = 4;
static const uint8_t motors_right_pwm = 5;
static const uint8_t motors_left_dir = 6;
static const uint8_t motors_right_dir= 7;
static const uint8_t motors_forward = 1;
static const uint8_t motors_backward = 0;

static const char* motors_file = "/home/pi/mmap_buffors/motors_buffor";
static int motors_fd = -1;

/* Current motors state */
motors_t* motors = 0;

/* Initialize motors subsystem */
int motors_init(void)
{
	/* Initialize I2C bus */
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

	/* Perform mapping for exchanging values */
	if (shared_memory_map_rdwr(motors_file, &motors_fd, (void*) &motors,
							 sizeof(motors_t)) < 0)
	{
		syslog(LOG_ERR, "Cannot initialize motors subsystem.");
		motors_destroy();
		return -1;
	}

	return 0;
}

/* Shutdown motors subsystem */
void motors_destroy(void)
{
	close(motors_i2c_fd);
	motors_i2c_fd = -1;
	shared_memory_unmap(&motors_fd, (void**) &motors, sizeof(motors_t));
}

/* Set motors' control values */
void motors_write(int left, int right)
{
	/* Split value [-100;100] into absolute value and sign, write to register */
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

	/* Inform other applications */
	motors->left = left;
	motors->right = right;
}
