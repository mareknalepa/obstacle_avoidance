#include "pathfinder_a1.h"

#include "common.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "steering.h"
#include "brake.h"
#include <math.h>

typedef enum {
    PATHFINDER_A1_NONE,
    PATHFINDER_A1_LOOKUP_LEFT_ROTATE,
    PATHFINDER_A1_LOOKUP_RIGHT_ROTATE,
    PATHFINDER_A1_LOOKUP_RETURN,
    PATHFINDER_A1_PROCESS_SAMPLES,
    PATHFINDER_A1_SET_HEADING,
    PATHFINDER_A1_DRIVE,
    PATHFINDER_A1_FINISH,
} pathfinder_a1_mode_t;

pathfinder_a1_mode_t pathfinder_a1_mode = PATHFINDER_A1_NONE;

#define MAX_HEADING 80
#define MAX_HEADING_RATE_SEARCH 3.5
#define MAX_HEADING_RATE_NORMAL 8.0
#define PATHFINDER_A1_DIST 30
#define HEADING_PREFER_BASE 1.25
#define HEADING_PREFER_FACTOR 0.25

#define MAX_SAMPLES 48
static double angles[MAX_SAMPLES];
static int distances[MAX_SAMPLES];
static double weights[MAX_SAMPLES];

static int initial_obstacle_distance = 0;
static double prev_dist = 0.0;
static double position_x = 0.0;
static double position_y = 0.0;
static double heading_sin = 0.0;
static double heading_cos = 1.0;

static inline void pathfinder_a1_collect_distance(void)
{
	int index = (sensors_data.heading + 60) * MAX_SAMPLES / (MAX_HEADING * 2);
	if (index < 0)
		index = 0;
	else if (index >= MAX_SAMPLES)
		index = MAX_SAMPLES - 1;
	angles[index] = sensors_data.heading;
	distances[index] = sensors_data.distance;
}

static inline void pathfinder_a1_filter_distances(void)
{
	double angles_step = (MAX_HEADING * 2.0) / (MAX_SAMPLES - 1);

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

		if (angles[i] == 0.0)
			angles[i] = -MAX_HEADING + i * angles_step;
	}
}

static inline void pathfinder_a1_calculate_weights(void)
{
	weights[0] = (distances[0] * 3 + distances[1] * 2 + distances[2]) / 6;
	weights[1] = (distances[0] * 2 + distances[1] * 3 + distances[2] * 2 +
		distances[3]) / 8;
	for (int i = 2; i < (MAX_SAMPLES - 2); ++i)
	{
		weights[i] = (distances[i - 2] + distances[i - 1] * 2 +
			distances[i] * 3 + distances[i + 1] * 2 + distances[i + 2]) / 9;
	}
	weights[MAX_SAMPLES - 2] = (distances[MAX_SAMPLES - 4] +
		distances[MAX_SAMPLES - 3] * 2 + distances[MAX_SAMPLES - 2] * 3 +
		distances[MAX_SAMPLES - 1] * 2) / 8;
	weights[MAX_SAMPLES - 1] = (distances[MAX_SAMPLES - 3] +
		distances[MAX_SAMPLES - 2] * 2 + distances[MAX_SAMPLES - 1] * 3) / 6;

	/* Prefer headings towards planned route */
	if (fabs(position_x) > 1.0)
	{
		double multiplier = -1.0;
		if (position_x < 0.0)
			multiplier = 1.0;

		for (int i = 0; i < MAX_SAMPLES; ++i)
		{
			double ratio = HEADING_PREFER_BASE +
				(angles[i] / MAX_HEADING * multiplier) * HEADING_PREFER_FACTOR;
			weights[i] *= ratio;
		}
	}
}

void pathfinder_a1_action(void)
{
	switch (pathfinder_a1_mode)
	{
	case PATHFINDER_A1_NONE:
		sensors_data.heading = 0;
		sensors_data_reset_odo();
		initial_obstacle_distance = sensors_data.distance;
		memset((void*) distances, 0, sizeof(distances));
		memset((void*) weights, 0, sizeof(weights));
		position_x = 0.0;
		position_y = 0.0;
		heading_sin = 0.0;
		heading_cos = 1.0;
		steering.mode = STEERING_ROTATE;
		steering.desired_heading = -MAX_HEADING;
		steering.max_heading_rate = MAX_HEADING_RATE_SEARCH;
		pathfinder_a1_mode = PATHFINDER_A1_LOOKUP_LEFT_ROTATE;
		syslog(LOG_INFO, "Pathfinder A1: sampling environment...");
		break;
	case PATHFINDER_A1_LOOKUP_LEFT_ROTATE:
		pathfinder_a1_collect_distance();

		if (steering.mode == STEERING_STOP)
		{
			steering.mode = STEERING_ROTATE;
			steering.desired_heading = MAX_HEADING;
			steering.max_heading_rate = MAX_HEADING_RATE_SEARCH;
			pathfinder_a1_mode = PATHFINDER_A1_LOOKUP_RIGHT_ROTATE;
		}
		break;
	case PATHFINDER_A1_LOOKUP_RIGHT_ROTATE:
		pathfinder_a1_collect_distance();

		if (steering.mode == STEERING_STOP)
		{
			steering.mode = STEERING_ROTATE;
			steering.desired_heading = 0.0;
			steering.max_heading_rate = MAX_HEADING_RATE_SEARCH;
			pathfinder_a1_mode = PATHFINDER_A1_LOOKUP_RETURN;
		}
		break;
	case PATHFINDER_A1_LOOKUP_RETURN:
		pathfinder_a1_collect_distance();

		if (steering.mode == STEERING_STOP)
			pathfinder_a1_mode = PATHFINDER_A1_PROCESS_SAMPLES;
		break;
	case PATHFINDER_A1_PROCESS_SAMPLES:
		pathfinder_a1_filter_distances();
		pathfinder_a1_calculate_weights();

		int best_index = 0;
		for (int i = 0; i < MAX_SAMPLES; ++i)
		{
			if (weights[i] > weights[best_index])
				best_index = i;
		}
		steering.mode = STEERING_ROTATE;
		steering.desired_heading = angles[best_index];
		steering.max_heading_rate = MAX_HEADING_RATE_NORMAL;

		syslog(LOG_INFO, "Pathfinder A1: choosing heading %.2f",
			 steering.desired_heading);
		pathfinder_a1_mode = PATHFINDER_A1_SET_HEADING;
		break;
	case PATHFINDER_A1_SET_HEADING:
		if (steering.mode == STEERING_STOP)
		{
			sensors_data_reset_odo();
			prev_dist = 0.0;
			heading_sin = sin(steering.desired_heading * M_PI / 180.0);
			heading_cos = cos(steering.desired_heading * M_PI / 180.0);
			steering.mode = STEERING_DRIVE_FORWARD;
			steering.desired_odo = PATHFINDER_A1_DIST;
			steering.desired_space = 25;
			pathfinder_a1_mode = PATHFINDER_A1_DRIVE;
		}
		break;
	case PATHFINDER_A1_DRIVE:
		position_x += heading_sin * (sensors_data.odo - prev_dist);
		position_y += heading_cos * (sensors_data.odo - prev_dist);
		prev_dist = sensors_data.odo;

		if (steering.mode == STEERING_STOP)
		{
			sensors_data_reset_odo();
			prev_dist = 0.0;
			memset((void*) distances, 0, sizeof (distances));
			memset((void*) weights, 0, sizeof (weights));

			steering.mode = STEERING_ROTATE;
			steering.desired_heading = -MAX_HEADING;
			steering.max_heading_rate = MAX_HEADING_RATE_SEARCH;
			pathfinder_a1_mode = PATHFINDER_A1_LOOKUP_LEFT_ROTATE;
			syslog(LOG_INFO, "Pathfinder A1: sampling environment...");
		}

		if (fabs(position_x) < 2 && position_y > initial_obstacle_distance)
		{
			steering.mode = STEERING_ROTATE;
			steering.desired_heading = 0.0;
			steering.max_heading_rate = MAX_HEADING_RATE_NORMAL;
			pathfinder_a1_mode = PATHFINDER_A1_FINISH;
		}
		break;
	case PATHFINDER_A1_FINISH:
		if (steering.mode == STEERING_STOP)
		{
			pathfinder_a1_mode = PATHFINDER_A1_NONE;
			mode_switch(MODE_SUPERVISOR);
			if (ipc_raspberry_daemon_attach() < 0)
				syslog(LOG_ERR, "Cannot enter supervisor mode.");
		}
		break;
	}
}
