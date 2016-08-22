#include "supervisor.h"

#include "common.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "brake.h"
#include "motors.h"
#include <time.h>

static double supervisor_time = 0.0;
static int supervisor_dist = 0;

void supervisor_action(void)
{
	if (motors->left == 100 && motors->right == 100 &&
		supervisor_time == 0.0 && sensors_data.dist->distance < 199)
	{
		struct timespec time;
		clock_gettime(CLOCK_MONOTONIC, &time);
		supervisor_time = time.tv_sec * 1000 + time.tv_nsec / 1000000;
		supervisor_dist = sensors_data.dist->distance;
	}

	if (motors->left != 100 || motors->right != 100)
	{
		supervisor_time = 0.0;
	}

	if ((motors->left > 0 || motors->right > 0) &&
		sensors_data.dist->distance < DISTANCE_TRESHOLD)
	{
		mode_switch(MODE_BRAKE);
		if (ipc_raspberry_daemon_detach() < 0)
			syslog(LOG_ERR, "Cannot enter brake mode.");

		if (supervisor_time != 0.0)
		{
			struct timespec time;
			clock_gettime(CLOCK_MONOTONIC, &time);
			double elapsed = (time.tv_sec * 1000 + time.tv_nsec / 1000000) -
				supervisor_time;
			if (elapsed > 0.1)
			{
				supervisor_time = 0.0;
				int distance_covered = supervisor_dist -
					sensors_data.dist->distance;

				sensors_data.dist_ratio = (distance_covered * 1000 / elapsed) /
					10.0;
			}
		}
	}
}
