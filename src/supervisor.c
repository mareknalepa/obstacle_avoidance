#include "supervisor.h"

#include "common.h"
#include "constants.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "brake.h"
#include "motors.h"

void supervisor_action(void)
{
	if ((motors->left > 0 || motors->right > 0) &&
		sensors_data.distance < DISTANCE_BRAKE)
	{
		mode_switch(MODE_BRAKE);
		if (ipc_raspberry_daemon_detach() < 0)
			syslog(LOG_ERR, "Cannot enter brake mode.");
	}
}
