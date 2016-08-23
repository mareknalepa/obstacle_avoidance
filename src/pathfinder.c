#include "pathfinder.h"

#include "common.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "motors.h"
#include "brake.h"
#include <math.h>

#define MOTORS_SPEED 70
#define MAX_HEADING 60
#define HEADING_OFFSET 5
#define PATHFINDER_DIST 25
#define HEADING_PREFER_BASE 1.25
#define HEADING_PREFER_FACTOR 0.25

pathfinder_mode_t pathfinder_mode = PATHFINDER_NONE;

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
static double current_heading = 0.0;
static double desired_heading = 0.0;

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

static inline void pathfinder_calculate_weights(void)
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

void pathfinder_action(void)
{
	switch (pathfinder_mode)
	{
	case PATHFINDER_NONE:
		sensors_data.heading = 0;
		initial_obstacle_distance = sensors_data.dist->distance;
		memset((void*) distances, 0, sizeof(distances));
		memset((void*) weights, 0, sizeof(weights));
		position_x = 0.0;
		position_y = 0.0;
		heading_sin = 0.0;
		heading_cos = 1.0;
		motors_write(-MOTORS_SPEED, MOTORS_SPEED);
		pathfinder_mode = PATHFINDER_LOOKUP_LEFT_ROTATE;
		syslog(LOG_INFO, "Pathfinder: sampling environment...");
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

		if (sensors_data.heading >= -HEADING_OFFSET)
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

		if (sensors_data.heading <= HEADING_OFFSET)
		{
			motors_write(0, 0);
			sensors_data.heading = 0.0;
			pathfinder_mode = PATHFINDER_PROCESS_SAMPLES;
		}
		break;
	case PATHFINDER_PROCESS_SAMPLES:
		pathfinder_filter_distances();
		pathfinder_calculate_weights();

		int best_index = 0;
		for (int i = 0; i < MAX_SAMPLES; ++i)
		{
			if (weights[i] > weights[best_index])
				best_index = i;
		}
		desired_heading = angles[best_index];

		syslog(LOG_INFO, "Pathfinder: choosing heading %.2f", desired_heading);

		if (desired_heading < sensors_data.heading)
			motors_write(-MOTORS_SPEED, MOTORS_SPEED);
		else
			motors_write(MOTORS_SPEED, -MOTORS_SPEED);
		pathfinder_mode = PATHFINDER_SET_HEADING;
		break;
	case PATHFINDER_SET_HEADING:
		if (fabs(desired_heading - sensors_data.heading) <= HEADING_OFFSET)
		{
			motors_write(0, 0);
			sensors_data.dist_traveled = 0.0;
			prev_dist = 0.0;
			current_heading = desired_heading;
			heading_sin = sin(current_heading * M_PI / 180.0);
			heading_cos = cos(current_heading * M_PI / 180.0);
			motors_write(MOTORS_SPEED, MOTORS_SPEED);
			pathfinder_mode = PATHFINDER_DRIVE;
		}
		break;
	case PATHFINDER_DRIVE:
		position_x += heading_sin * (sensors_data.dist_traveled - prev_dist);
		position_y += heading_cos * (sensors_data.dist_traveled - prev_dist);
		prev_dist = sensors_data.dist_traveled;
		
		if (sensors_data.dist->distance <= 10)
			motors_write(0, 0);

		if (sensors_data.dist_traveled >= PATHFINDER_DIST)
		{
			motors_write(0, 0);
			desired_heading = 0.0;

			if (desired_heading < sensors_data.heading)
				motors_write(-MOTORS_SPEED, MOTORS_SPEED);
			else
				motors_write(MOTORS_SPEED, -MOTORS_SPEED);
			pathfinder_mode = PATHFINDER_RESET_HEADING;
		}
		
		if (fabs(position_x) < 2 && position_y > initial_obstacle_distance)
		{
			motors_write(0, 0);
			desired_heading = 0.0;
			
			if (desired_heading < sensors_data.heading)
				motors_write(-MOTORS_SPEED, MOTORS_SPEED);
			else
				motors_write(MOTORS_SPEED, -MOTORS_SPEED);
			pathfinder_mode = PATHFINDER_FINISH;
		}
		break;
	case PATHFINDER_RESET_HEADING:
		if (fabs(desired_heading - sensors_data.heading) <= HEADING_OFFSET)
		{
			motors_write(0, 0);
			sensors_data.dist_traveled = 0.0;
			prev_dist = 0.0;

			sensors_data.heading = 0;
			memset((void*) distances, 0, sizeof (distances));
			memset((void*) weights, 0, sizeof (weights));
			motors_write(-MOTORS_SPEED, MOTORS_SPEED);
			pathfinder_mode = PATHFINDER_LOOKUP_LEFT_ROTATE;
			syslog(LOG_INFO, "Pathfinder: sampling environment...");
		}
		break;
	case PATHFINDER_FINISH:
		if (fabs(desired_heading - sensors_data.heading) <= HEADING_OFFSET)
		{
			motors_write(0, 0);
			pathfinder_mode = PATHFINDER_NONE;
			mode_switch(MODE_SUPERVISOR);
			if (ipc_raspberry_daemon_attach() < 0)
				syslog(LOG_ERR, "Cannot enter supervisor mode.");
		}
		break;
	}
}
