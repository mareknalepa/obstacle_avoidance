#include "pathfinder.h"

#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "motors.h"
#include "brake.h"

void pathfinder_action(void)
{
	if (sensors_data.dist->distance > DISTANCE_TRESHOLD)
	{
		mode_switch(MODE_SUPERVISOR);
		if (ipc_raspberry_daemon_attach() < 0)
			syslog(LOG_ERR, "Cannot enter supervisor mode.");
	}
}
