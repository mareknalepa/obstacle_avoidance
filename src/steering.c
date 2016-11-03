#include "steering.h"

#include "common.h"
#include "constants.h"
#include "motors.h"
#include "sensors_data.h"
#include <math.h>

steering_t steering = { 0 };

static double raw_steering = 0.0;
static double distance_err = 0.0;
static double drive_steering = 0.0;

static double heading_rate_setpoint = 0.0;
static double heading_rate_err = 0.0;
static double heading_rate_prev = 0.0;
static double heading_rate_err_sum = 0.0;
static double heading_rate_steering = 0.0;

static double heading_err = 0.0;
static double heading_prev = 0.0;
static double heading_err_sum = 0.0;

static double steering_trunc(double value, double min, double max)
{
	if (value > max)
		return max;
	else if (value < min)
		return min;
	else
		return value;
}

void steering_cycle(void)
{
	switch (steering.mode)
	{
	case STEERING_STOP:
		steering.max_heading_rate = HEADING_MAX_OUT_DEFAULT;
		break;
	case STEERING_DRIVE_FORWARD:
		distance_err = steering.desired_odo - sensors_data.odo;
		if (distance_err > sensors_data.distance - LONGITUDINAL_CLEARANCE)
		{
			if (sensors_data.distance > LONGITUDINAL_CLEARANCE)
				raw_steering = (sensors_data.distance -
					LONGITUDINAL_CLEARANCE) * DRIVE_P;
			else
				raw_steering = 0.0;
		}
		else
		{
			raw_steering = distance_err * DRIVE_P;
		}
		drive_steering = steering_trunc(distance_err * DRIVE_P,
			-DRIVE_MAX_OUT, DRIVE_MAX_OUT);

		if (fabs(distance_err) < 2.0)
		{
			motors_write(0, 0);
			steering.mode = STEERING_STOP;
		}
		else
		{
			motors_write(drive_steering, drive_steering);
		}
		break;
	case STEERING_ROTATE:
		/* Outside PID controller (controls heading change rate) */
		heading_err = steering.desired_heading - sensors_data.heading;
		heading_rate_setpoint = HEADING_P * heading_err;

		heading_err_sum += heading_err;
		heading_err_sum = steering_trunc(heading_err_sum, -HEADING_SUM_MAX,
			HEADING_SUM_MAX);
		heading_rate_setpoint += HEADING_I * heading_err_sum;

		heading_rate_setpoint += HEADING_D *
			(heading_prev - sensors_data.heading);
		heading_prev = sensors_data.heading;

		heading_rate_setpoint = steering_trunc(heading_rate_setpoint,
			-steering.max_heading_rate, steering.max_heading_rate);

		/* Inside PID controller (controls actual rotating) */
		heading_rate_err = heading_rate_setpoint - sensors_data.heading_rate;
		heading_rate_steering = HEADING_RATE_P * heading_rate_err;

		heading_rate_err_sum += heading_rate_err;
		heading_rate_err_sum = steering_trunc(heading_rate_err_sum,
			-HEADING_RATE_SUM_MAX, HEADING_RATE_SUM_MAX);
		heading_rate_steering += HEADING_RATE_I * heading_rate_err_sum;

		heading_rate_steering += HEADING_RATE_D *
			(heading_rate_prev - sensors_data.heading_rate);
		heading_rate_prev = sensors_data.heading_rate;

		heading_rate_steering = steering_trunc(heading_rate_steering,
			-HEADING_RATE_MAX_OUT, HEADING_RATE_MAX_OUT);

		if (fabs(heading_err) < 2.0)
		{
			motors_write(0, 0);
			steering.mode = STEERING_STOP;
			heading_err_sum = 0.0;
			heading_rate_err_sum = 0.0;
		}
		else
		{
			motors_write(heading_rate_steering, -heading_rate_steering);
		}
		break;
	}
}
