/*
 * pathfinder_a2.c
 *
 * Author: Marek Nalepa
 * Purpose: Implementation of A2 obstacle avoidance algorithm.
 */

#include "pathfinder_a2.h"

#include "common.h"
#include "constants.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "steering.h"
#include "brake.h"
#include "stats.h"
#include <math.h>

/* A2 algorithm possible states */
typedef enum {
    PATHFINDER_A2_NONE,
	PATHFINDER_A2_SEARCH_EDGE,
	PATHFINDER_A2_AVOID_EDGE,
	PATHFINDER_A2_DRIVE_FORWARD,
	PATHFINDER_A2_ROTATE_TO_CENTER,
	PATHFINDER_A2_FINISH,
} pathfinder_a2_mode_t;

pathfinder_a2_mode_t pathfinder_a2_mode = PATHFINDER_A2_NONE;

/* Internal variables */
static int rotate_dir = -1.0;
static int prev_distances[2];
static double prev_headings[2];
static int prev_index = 0;
static int samples_number = 0;
static int can_resume = 0;

/* Logic action handler */
void pathfinder_a2_action(void)
{
	switch (pathfinder_a2_mode)
	{
		/* Algorithm initialization */
	case PATHFINDER_A2_NONE:
		stats_start();
		sensors_data_reset_coordinates();
		rotate_dir = -1.0;
		prev_distances[0] = 0;
		prev_distances[1] = 0;
		prev_headings[0] = 0.0;
		prev_headings[1] = 0.0;
		prev_index = 0;
		samples_number = 0;
		can_resume = 0;

		steering_rotate(-MAX_HEADING, HEADING_RATE_LOOKUP);
		syslog(LOG_INFO, "Pathfinder A2: searching edge of obstacle (left)...");
		pathfinder_a2_mode = PATHFINDER_A2_SEARCH_EDGE;
		break;
		/* Rotating and looking for obstacle edge */
	case PATHFINDER_A2_SEARCH_EDGE:
		if (rotate_dir == -1.0 && steering.mode == STEERING_STOP)
		{
			syslog(LOG_INFO, "Pathfinder A2: searching edge of obstacle "
				"(right)...");
			steering_rotate(MAX_HEADING, HEADING_RATE_LOOKUP);
			rotate_dir = 1.0;
		}
		if (rotate_dir == 1.0 && steering.mode == STEERING_STOP)
		{
			syslog(LOG_INFO, "Pathfinder A2: reached max right heading.");
			steering_rotate(0.0, HEADING_RATE_TURN);
			rotate_dir = -1.0;
			pathfinder_a2_mode = PATHFINDER_A2_FINISH;
			break;
		}
		if (samples_number > 2)
		{
			double x1 = prev_distances[!prev_index] *
				sin(prev_headings[!prev_index] * M_PI / 180.0);
			double y1 = prev_distances[!prev_index] *
				cos(prev_headings[!prev_index] * M_PI / 180.0);
			double x2 = prev_distances[prev_index] *
				sin(prev_headings[prev_index] * M_PI / 180.0);
			double y2 = prev_distances[prev_index] *
				cos(prev_headings[prev_index] * M_PI / 180.0);

			double a = (y2 - y1) / (x2 - x1);
			double b = y1 - a * x1;

			double y3 = b / (1.0 - a * tan(sensors_data.heading * M_PI /
				180.0));

			double predicted_distance = y3 / cos(sensors_data.heading * M_PI /
				180.0);
			double tolerance = predicted_distance * PATHFINDER_A2_TOLERANCE;

			if (fabs(predicted_distance - sensors_data.distance)
				> tolerance && sensors_data.distance > 40)
			{
				syslog(LOG_INFO, "Pathfinder A2: found edge at "
					"direction %.1f.", sensors_data.heading);

				double clearance_heading = fabs(180.0 / M_PI *
					asin((VEHICLE_WIDTH / 2.0 + TRANSVERSE_CLEARANCE) /
					sensors_data.distance));

				if (rotate_dir == -1)
					steering_rotate(sensors_data.heading - clearance_heading,
								 HEADING_RATE_TURN);
				else
					steering_rotate(sensors_data.heading + clearance_heading,
								 HEADING_RATE_TURN);
				pathfinder_a2_mode = PATHFINDER_A2_AVOID_EDGE;
				break;
			}
		}

		prev_index = !prev_index;
		prev_distances[prev_index] = sensors_data.distance;
		prev_headings[prev_index] = sensors_data.heading;
		++samples_number;
		break;
		/* Rotate a bit more to drive with sufficient clearance */
	case PATHFINDER_A2_AVOID_EDGE:
		if (steering.mode == STEERING_STOP)
		{
			steering_drive(prev_distances[prev_index] + VEHICLE_LENGTH * 2.0);
			++stats.heading_changes;
			++stats.forward_rides;
			pathfinder_a2_mode = PATHFINDER_A2_DRIVE_FORWARD;
		}
		break;
		/* Drive forward next to edge */
	case PATHFINDER_A2_DRIVE_FORWARD:
		if (steering.mode == STEERING_STOP)
		{
			if (fabs(sensors_data.odo - steering.desired_odo) < 5.0)
				syslog(LOG_INFO, "Pathfinder A2: reached obstacle edge.");
			else
				syslog(LOG_INFO, "Pathfinder A2: stopped in front of obstacle.");

			if (sensors_data.position_x < 0)
				steering_rotate(MAX_HEADING, HEADING_RATE_TURN);
			else
				steering_rotate(-MAX_HEADING, HEADING_RATE_TURN);
			pathfinder_a2_mode = PATHFINDER_A2_ROTATE_TO_CENTER;
		}

		if (can_resume && fabs(sensors_data.position_x) < 2.0)
		{
			steering_rotate(0.0, HEADING_RATE_TURN);

			if (sensors_data.heading > 0.0)
				rotate_dir = -1.0;
			else
				rotate_dir = 1.0;
			pathfinder_a2_mode = PATHFINDER_A2_FINISH;
		}
		break;
		/* Rotate towards initial vehicle's direction */
	case PATHFINDER_A2_ROTATE_TO_CENTER:
		if (steering.mode == STEERING_STOP)
		{
			can_resume = 1;
			++stats.heading_changes;
			syslog(LOG_INFO, "Pathfinder A2: Rotated to center.");

			double distance_to_course = fabs(sensors_data.position_x /
				cos((90.0 - sensors_data.heading) * M_PI / 180.0));
			if (sensors_data.distance > distance_to_course)
			{
				steering_drive(distance_to_course + VEHICLE_LENGTH);
				pathfinder_a2_mode = PATHFINDER_A2_DRIVE_FORWARD;
				++stats.forward_rides;
			}
			else
			{
				if (sensors_data.position_x < 0)
					rotate_dir = -1.0;
				else
					rotate_dir = 1.0;

				prev_distances[0] = sensors_data.distance;
				prev_distances[1] = 0;
				prev_headings[0] = sensors_data.heading;
				prev_headings[1] = 0.0;
				prev_index = 0;
				samples_number = 1;
				steering_rotate(rotate_dir * MAX_HEADING, HEADING_RATE_LOOKUP);
				pathfinder_a2_mode = PATHFINDER_A2_SEARCH_EDGE;
			}
		}
		break;
		/* Quit obstacle avoidance, return control to main daemon */
	case PATHFINDER_A2_FINISH:
		if (steering.mode == STEERING_STOP)
		{
			++stats.heading_changes;
			stats_end();
			pathfinder_a2_mode = PATHFINDER_A2_NONE;
			mode_switch(MODE_SUPERVISOR);
			if (ipc_raspberry_daemon_attach() < 0)
				syslog(LOG_ERR, "Cannot enter supervisor mode.");
		}
		break;
	}
}
