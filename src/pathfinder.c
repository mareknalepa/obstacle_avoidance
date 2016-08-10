#include "pathfinder.h"

#include "common.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "motors.h"
#include "brake.h"

#define MOTORS_SPEED 70
#define MAX_HEADING 60

pathfinder_mode_t pathfinder_mode = PATHFINDER_NONE;

#define MAX_SAMPLES 48
static double angles[MAX_SAMPLES];
static int distances[MAX_SAMPLES];

static inline void pathfinder_collect_distance(void)
{
	int index = (sensors_data.heading + 60) * MAX_SAMPLES / (MAX_HEADING * 2);
	if (index < 0)
		index = 0;
	else if (index >= MAX_SAMPLES)
		index = MAX_SAMPLES - 1;
	angles[index] = sensors_data.heading;
	distances[index] = sensors_data.dist->distance;
}

static inline void pathfinder_filter_distances(void)
{
	for (int i = 0; i < MAX_SAMPLES; ++i)
	{
		if (distances[i] == 0)
		{
			if (i == 0)
				distances[0] = distances[1];
			else if (i == (MAX_SAMPLES - 1))
				distances[MAX_SAMPLES - 1] = distances[MAX_SAMPLES - 2];
			else
				distances[i] = (distances[i - 1] + distances[i + 1]) / 2;
		}
		else if (distances[i] > 200)
			distances[i] = 200;
	}
}

void pathfinder_action(void)
{
	switch (pathfinder_mode)
	{
	case PATHFINDER_NONE:
		sensors_data.heading = 0;
		memset((void*) distances, 0, sizeof(distances));
		motors_write(-MOTORS_SPEED, MOTORS_SPEED);
		pathfinder_mode = PATHFINDER_LOOKUP_LEFT_ROTATE;
		break;
	case PATHFINDER_LOOKUP_LEFT_ROTATE:
		pathfinder_collect_distance();

		if (sensors_data.heading <= -MAX_HEADING)
		{
			motors_write(0, 0);
			motors_write(MOTORS_SPEED, -MOTORS_SPEED);
			pathfinder_mode = PATHFINDER_LOOKUP_LEFT_RETURN;
		}
		break;
	case PATHFINDER_LOOKUP_LEFT_RETURN:
		pathfinder_collect_distance();

		if (sensors_data.heading >= -1.5)
		{
			pathfinder_mode = PATHFINDER_LOOKUP_RIGHT_ROTATE;
		}
		break;
	case PATHFINDER_LOOKUP_RIGHT_ROTATE:
		pathfinder_collect_distance();

		if (sensors_data.heading >= MAX_HEADING)
		{
			motors_write(0, 0);
			motors_write(-MOTORS_SPEED, MOTORS_SPEED);
			pathfinder_mode = PATHFINDER_LOOKUP_RIGHT_RETURN;
		}
		break;
	case PATHFINDER_LOOKUP_RIGHT_RETURN:
		pathfinder_collect_distance();

		if (sensors_data.heading <= 1.5)
		{
			motors_write(0, 0);
			pathfinder_mode = PATHFINDER_SET_HEADING;
		}
		break;
	case PATHFINDER_SET_HEADING:
		pathfinder_filter_distances();
		pathfinder_mode = PATHFINDER_DRIVE;
		break;
	default:
		motors_write(0, 0);
		pathfinder_mode = PATHFINDER_NONE;
		mode_switch(MODE_SUPERVISOR);
		if (ipc_raspberry_daemon_attach() < 0)
			syslog(LOG_ERR, "Cannot enter supervisor mode.");
		break;
	}
}
