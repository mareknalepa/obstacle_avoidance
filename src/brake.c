#include "brake.h"

#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "motors.h"

static int brake_initial_left = 0;
static int brake_initial_right = 0;

void brake_action(void)
{
	if (brake_initial_left == 0 || brake_initial_right == 0)
	{
		motors_read(&brake_initial_left, &brake_initial_right);

		/* TODO: fix invalid motors' PWM values */
		brake_initial_left = 100;
		brake_initial_right = 100;
	}
	
	int distance = sensors_data.dist->distance;
	float ratio = ((float) (distance - DISTANCE_STOP) / DISTANCE_DIFF);
	
	if (ratio >= 1.0f)
	{
		brake_initial_left = 0;
		brake_initial_right = 0;
		mode_switch(MODE_SUPERVISOR);
		if (ipc_raspberry_daemon_attach() < 0)
			syslog(LOG_ERR, "Cannot enter supervisor mode.");
	}
	else if (ratio > 0.1f)
	{
		int left = brake_initial_left * ratio;
		int right = brake_initial_right * ratio;
		motors_write(left, right);
	}
	else
	{
		motors_write(0, 0);
		brake_initial_left = 0;
		brake_initial_right = 0;
		mode_switch(MODE_PATHFINDER);
	}
}
