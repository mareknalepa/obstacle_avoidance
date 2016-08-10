#include "supervisor.h"

#include "common.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "brake.h"
#include "motors.h"

void supervisor_action(void)
{
	int left, right;
	motors_read(&left, &right);

	if (left != 0 && right != 0 &&
		sensors_data.dist->distance < DISTANCE_TRESHOLD)
	{
		mode_switch(MODE_BRAKE);
		if (ipc_raspberry_daemon_detach() < 0)
			syslog(LOG_ERR, "Cannot enter brake mode.");
	}
}
