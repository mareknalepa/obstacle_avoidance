#include "motors.h"

#include "common.h"
#include <linux/i2c-dev.h>

static const char* motors_i2c_path = "/dev/i2c-1";
static const int motors_i2c_address = 0x47;
static int motors_i2c_fd = -1;

static const uint8_t motors_left_pwm = 4;
static const uint8_t motors_right_pwm = 5;
static const uint8_t motors_left_dir = 6;
static const uint8_t motors_right_dir= 7;
static const uint8_t motors_forward = 1;
static const uint8_t motors_backward = 0;

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
	
	return 0;
}

void motors_destroy(void)
{
	close(motors_i2c_fd);
	motors_i2c_fd = -1;
}

void motors_read(int* left, int* right)
{
	*left = i2c_smbus_read_byte_data(motors_i2c_fd, motors_left_pwm);
	int direction = (int) i2c_smbus_read_byte_data(motors_i2c_fd,
												 motors_left_dir);
	if (direction == motors_backward)
		*left *= -1;
	
	*right = i2c_smbus_read_byte_data(motors_i2c_fd, motors_right_pwm);
	direction = (int) i2c_smbus_read_byte_data(motors_i2c_fd, motors_right_dir);
	if (direction == motors_backward)
		*right *= -1;
}

void motors_write(int left, int right)
{
	uint8_t byte;
	if (left < 0)
	{
		byte = (uint8_t) (left * -1);
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_left_pwm, byte);
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_left_dir,
								 motors_backward);
	}
	else
	{
		byte = (uint8_t) left;
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_left_pwm, byte);
	}
	if (right < 0)
	{
		byte = (uint8_t) (right * -1);
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_right_pwm, byte);
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_right_dir,
								 motors_backward);
	}
	else
	{
		byte = (uint8_t) right;
		i2c_smbus_write_byte_data(motors_i2c_fd, motors_right_pwm, byte);
	}
}
