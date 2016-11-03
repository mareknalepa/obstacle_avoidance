/*
 * supervisor.c
 *
 * Author: Marek Nalepa
 * Purpose: Detect obstacles and enter brake mode.
 */

#include "supervisor.h"

#include "common.h"
#include "constants.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "motors.h"

/* Callback for logic actions while supervising ride */
void supervisor_action(void)
{
	/* If vehicle drives and obstacle is detected, start braking */
	if ((motors->left > 0 || motors->right > 0) &&
		sensors_data.distance < DISTANCE_BRAKE)
	{
		mode_switch(MODE_BRAKE);
		if (ipc_raspberry_daemon_detach() < 0)
			syslog(LOG_ERR, "Cannot enter brake mode.");
	}
}
