/*
 * pathfinder_a1.c
 *
 * Author: Marek Nalepa
 * Purpose: Implementation of A1 obstacle avoidance algorithm.
 */

#include "pathfinder_a1.h"

#include "common.h"
#include "constants.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "steering.h"
#include "brake.h"
#include "stats.h"
#include <math.h>

/* A1 algorithm possible states */
typedef enum {
	PATHFINDER_A1_NONE,
	PATHFINDER_A1_TURN_LEFT,
	PATHFINDER_A1_LOOKUP,
	PATHFINDER_A1_PROCESS_SAMPLES,
	PATHFINDER_A1_SET_HEADING,
	PATHFINDER_A1_DRIVE_FORWARD,
	PATHFINDER_A1_FINISH,
} pathfinder_a1_mode_t;

pathfinder_a1_mode_t pathfinder_a1_mode = PATHFINDER_A1_NONE;

/* Internal variables */
static double headings[PATHFINDER_A1_SAMPLES];
static int distances[PATHFINDER_A1_SAMPLES];
static double weights[PATHFINDER_A1_SAMPLES];
static int initial_obstacle_distance = 0;

/* Sample environment and store values in array */
static inline void pathfinder_a1_collect_distance(void)
{
	/* Calculate index based on heading */
	int index = (sensors_data.heading + 60) * PATHFINDER_A1_SAMPLES /
		(MAX_HEADING * 2);
	if (index < 0)
		index = 0;
	else if (index >= PATHFINDER_A1_SAMPLES)
		index = PATHFINDER_A1_SAMPLES - 1;

	/* Store heading and distance */
	headings[index] = sensors_data.heading;
	distances[index] = sensors_data.distance;
}

/* Filter collected samples */
static inline void pathfinder_a1_filter_distances(void)
{
	double angles_step = (MAX_HEADING * 2.0) / (PATHFINDER_A1_SAMPLES - 1);

	/* If array element is empty, approximate it */
	for (int i = 0; i < PATHFINDER_A1_SAMPLES; ++i)
	{
		if (distances[i] == 0)
		{
			if (i == 0)
				distances[0] = distances[1];
			else if (i == (PATHFINDER_A1_SAMPLES - 1))
				distances[PATHFINDER_A1_SAMPLES - 1] =
					distances[PATHFINDER_A1_SAMPLES - 2];
			else
				distances[i] = (distances[i - 1] + distances[i + 1]) / 2;
		}
		else if (distances[i] > PATHFINDER_A1_MAX_DISTANCE)
			distances[i] = PATHFINDER_A1_MAX_DISTANCE;

		if (headings[i] == 0.0)
			headings[i] = -MAX_HEADING + i * angles_step;
	}
}

/* Calculate weights for particular headings based on distances */
static inline void pathfinder_a1_calculate_weights(void)
{
	/* Weight for heading is weighted average from adjacent headings */
	weights[0] = (distances[0] * 3 + distances[1] * 2 + distances[2]) / 6;
	weights[1] = (distances[0] * 2 + distances[1] * 3 + distances[2] * 2 +
		distances[3]) / 8;
	for (int i = 2; i < (PATHFINDER_A1_SAMPLES - 2); ++i)
	{
		weights[i] = (distances[i - 2] + distances[i - 1] * 2 +
			distances[i] * 3 + distances[i + 1] * 2 + distances[i + 2]) / 9;
	}
	weights[PATHFINDER_A1_SAMPLES - 2] = (distances[PATHFINDER_A1_SAMPLES - 4] +
		distances[PATHFINDER_A1_SAMPLES - 3] * 2 +
		distances[PATHFINDER_A1_SAMPLES - 2] * 3 +
		distances[PATHFINDER_A1_SAMPLES - 1] * 2) / 8;
	weights[PATHFINDER_A1_SAMPLES - 1] = (distances[PATHFINDER_A1_SAMPLES - 3] +
		distances[PATHFINDER_A1_SAMPLES - 2] * 2 +
		distances[PATHFINDER_A1_SAMPLES - 1] * 3) / 6;

	/* Prefer headings towards planned route */
	if (fabs(sensors_data.position_x) > 1.0)
	{
		double multiplier = -1.0;
		if (sensors_data.position_x < 0.0)
			multiplier = 1.0;

		/* Multiply weights by quadratic function */
		for (int i = 0; i < PATHFINDER_A1_SAMPLES; ++i)
		{
			double heading = headings[i] * multiplier;
			double ratio;
				ratio = (PATHFINDER_A1_PREFER_A * heading +
					PATHFINDER_A1_PREFER_B) * heading + PATHFINDER_A1_PREFER_C;
			weights[i] *= ratio;
		}
	}
}

/* Find best heading (with max weight) */
static inline double pathfinder_a1_best_heading(void)
{
	int best_index = 0;
	for (int i = 0; i < PATHFINDER_A1_SAMPLES; ++i)
	{
		if (weights[i] > weights[best_index])
			best_index = i;
	}

	return headings[best_index];
}

/* Logic action handler */
void pathfinder_a1_action(void)
{
	switch (pathfinder_a1_mode)
	{
		/* Algorithm initialization */
	case PATHFINDER_A1_NONE:
		stats_start();
		sensors_data_reset_coordinates();
		initial_obstacle_distance = sensors_data.distance;

		steering_rotate(-MAX_HEADING, HEADING_RATE_TURN);
		pathfinder_a1_mode = PATHFINDER_A1_TURN_LEFT;
		break;
		/* Turning left to max left heading */
	case PATHFINDER_A1_TURN_LEFT:
		if (steering.mode == STEERING_STOP)
		{
			memset((void*) distances, 0, sizeof(distances));
			memset((void*) weights, 0, sizeof(weights));
			syslog(LOG_INFO, "Pathfinder A1: sampling environment...");
			pathfinder_a1_collect_distance();

			steering_rotate(MAX_HEADING, HEADING_RATE_LOOKUP);
			pathfinder_a1_mode = PATHFINDER_A1_LOOKUP;
		}
		break;
		/* Collecting distances from max left heading to max right heading */
	case PATHFINDER_A1_LOOKUP:
		pathfinder_a1_collect_distance();

		if (steering.mode == STEERING_STOP)
			pathfinder_a1_mode = PATHFINDER_A1_PROCESS_SAMPLES;
		break;
		/* Filter data, calculate weights and choose best heading */
	case PATHFINDER_A1_PROCESS_SAMPLES:
		pathfinder_a1_filter_distances();
		pathfinder_a1_calculate_weights();

		steering_rotate(pathfinder_a1_best_heading(), HEADING_RATE_TURN);
		++stats.heading_changes;
		syslog(LOG_INFO, "Pathfinder A1: choosing heading %.2f",
			 steering.desired_heading);
		pathfinder_a1_mode = PATHFINDER_A1_SET_HEADING;
		break;
		/* Rotate to best heading */
	case PATHFINDER_A1_SET_HEADING:
		if (steering.mode == STEERING_STOP)
		{
			double distance_to_course = fabs(sensors_data.position_x /
				sin((90.0 - sensors_data.heading) * M_PI / 180.0));
			if (sensors_data.position_y > initial_obstacle_distance &&
				sensors_data.distance > distance_to_course)
				steering_drive(distance_to_course + VEHICLE_LENGTH);
			else
				steering_drive(PATHFINDER_A1_DRIVING_LENGTH);
			++stats.forward_rides;
			pathfinder_a1_mode = PATHFINDER_A1_DRIVE_FORWARD;
		}
		break;
		/* Follow best heading */
	case PATHFINDER_A1_DRIVE_FORWARD:
		if (steering.mode == STEERING_STOP)
		{
			steering_rotate(-MAX_HEADING, HEADING_RATE_TURN);
			pathfinder_a1_mode = PATHFINDER_A1_TURN_LEFT;
		}

		if (fabs(sensors_data.position_x) < 2 &&
			sensors_data.position_y > initial_obstacle_distance)
		{
			steering_rotate(0.0, HEADING_RATE_TURN);
			pathfinder_a1_mode = PATHFINDER_A1_FINISH;
		}
		break;
		/* Quit obstacle avoidance, return control to main daemon */
	case PATHFINDER_A1_FINISH:
		if (steering.mode == STEERING_STOP)
		{
			++stats.heading_changes;
			stats_end();
			pathfinder_a1_mode = PATHFINDER_A1_NONE;
			mode_switch(MODE_SUPERVISOR);
			if (ipc_raspberry_daemon_attach() < 0)
				syslog(LOG_ERR, "Cannot enter supervisor mode.");
		}
		break;
	}
}
