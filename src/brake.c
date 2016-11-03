/*
 * brake.c
 *
 * Author: Marek Nalepa
 * Purpose: To control vehicle while braking in front of obstacle.
 */

#include "brake.h"

#include "common.h"
#include "constants.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "motors.h"

static int brake_initial_left = 0;
static int brake_initial_right = 0;

/* Callback for logic actions while braking */
void brake_action(void)
{
	/* If this is first stage of braking, remember initial motors' values */
	if (brake_initial_left == 0 || brake_initial_right == 0)
	{
		brake_initial_left = motors->left;
		brake_initial_right = motors->right;
	}

	/* Calculate ratio */
	float ratio = ((float) (sensors_data.distance - DISTANCE_STOP) /
		DISTANCE_DIFF);

	/* Ratio shows us we have plenty of space - abort braking */
	if (ratio >= 1.0f)
	{
		brake_initial_left = 0;
		brake_initial_right = 0;
		mode_switch(MODE_SUPERVISOR);
		if (ipc_raspberry_daemon_attach() < 0)
			syslog(LOG_ERR, "Cannot enter supervisor mode.");
	}
	/* Normal brake stage - apply decreased motors' power */
	else if (ratio > 0.1f)
	{
		int left = brake_initial_left * ratio;
		int right = brake_initial_right * ratio;
		motors_write(left, right);
	}
	/* Speed is low, distance is low - it is safe to completely stop motors */
	else
	{
		motors_write(0, 0);
		brake_initial_left = 0;
		brake_initial_right = 0;
		mode_switch(MODE_PATHFINDER);
	}
}
