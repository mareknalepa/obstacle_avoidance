#include "pathfinder_a2.h"

#include "common.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "steering.h"
#include "brake.h"
#include "stats.h"
#include <math.h>

typedef enum {
    PATHFINDER_A2_NONE,
	PATHFINDER_A2_SEARCH_EDGE,
	PATHFINDER_A2_AVOID_EDGE,
	PATHFINDER_A2_DRIVE_FORWARD,
	PATHFINDER_A2_ROTATE_TO_CENTER,
	PATHFINDER_A2_FINISH,
} pathfinder_a2_mode_t;

pathfinder_a2_mode_t pathfinder_a2_mode = PATHFINDER_A2_NONE;

#define MAX_HEADING 80
#define MAX_HEADING_RATE_SEARCH 3.5
#define MAX_HEADING_RATE_NORMAL 8.0
#define HEADING_OFFSET 5
#define VEHICLE_WIDTH 20
#define VEHICLE_LENGTH 20
#define OBSTACLE_CLEARANCE 5

static int rotate_dir = -1.0;
static int prev_distances[2];
static double prev_headings[2];
static int prev_index = 0;
static int samples_number = 0;
static int can_resume = 0;

void pathfinder_a2_action(void)
{
	switch (pathfinder_a2_mode)
	{
	case PATHFINDER_A2_NONE:
		stats_start();
		sensors_data_reset_coordinates();
		sensors_data_reset_odo();
		rotate_dir = -1.0;
		prev_distances[0] = 0;
		prev_distances[1] = 0;
		prev_headings[0] = 0.0;
		prev_headings[1] = 0.0;
		prev_index = 0;
		samples_number = 0;
		can_resume = 0;

		steering.mode = STEERING_ROTATE;
		steering.desired_heading = -MAX_HEADING;
		steering.max_heading_rate = MAX_HEADING_RATE_SEARCH;
		steering.desired_space = 25.0;
		pathfinder_a2_mode = PATHFINDER_A2_SEARCH_EDGE;
		syslog(LOG_INFO, "Pathfinder A2: searching edge of obstacle (left)...");
		break;
	case PATHFINDER_A2_SEARCH_EDGE:
		if (rotate_dir == -1.0 && steering.mode == STEERING_STOP)
		{
			steering.mode = STEERING_ROTATE;
			steering.desired_heading = MAX_HEADING;
			syslog(LOG_INFO, "Pathfinder A2: reached max left heading.");
			rotate_dir = 1.0;
		}
		if (rotate_dir == 1.0 && steering.mode == STEERING_STOP)
		{
			steering.mode = STEERING_ROTATE;
			steering.desired_heading = 0.0;
			syslog(LOG_INFO, "Pathfinder A2: reached max right heading.");
			rotate_dir = -1.0;
			pathfinder_a2_mode = PATHFINDER_A2_FINISH;
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
			double tolerance = predicted_distance * 0.1;

			if (fabs(predicted_distance - sensors_data.distance)
				> tolerance && sensors_data.distance > 40)
			{
				syslog(LOG_INFO, "Pathfinder A2: found edge at "
					"direction %.1f.", sensors_data.heading);

				double clearance_heading = fabs(180.0 / M_PI *
					asin((VEHICLE_WIDTH / 2.0 + OBSTACLE_CLEARANCE) /
					sensors_data.distance));

				if (rotate_dir == -1)
					steering.desired_heading = sensors_data.heading -
						clearance_heading;
				else
					steering.desired_heading = sensors_data.heading +
						clearance_heading;
				steering.max_heading_rate = MAX_HEADING_RATE_NORMAL;
				steering.desired_odo = prev_distances[prev_index] +
					VEHICLE_LENGTH;
				sensors_data_reset_odo();
				pathfinder_a2_mode = PATHFINDER_A2_AVOID_EDGE;
			}
		}

		prev_index = !prev_index;
		prev_distances[prev_index] = sensors_data.distance;
		prev_headings[prev_index] = sensors_data.heading;
		++samples_number;
		break;
	case PATHFINDER_A2_AVOID_EDGE:
		if (steering.mode == STEERING_STOP)
		{
			steering.mode = STEERING_DRIVE_FORWARD;
			sensors_data_reset_odo();
			pathfinder_a2_mode = PATHFINDER_A2_DRIVE_FORWARD;
		}
		break;
	case PATHFINDER_A2_DRIVE_FORWARD:
		if (steering.mode == STEERING_STOP)
		{
			sensors_data_reset_odo();
			pathfinder_a2_mode = PATHFINDER_A2_ROTATE_TO_CENTER;
			if (fabs(sensors_data.odo - steering.desired_odo) < 5.0)
				syslog(LOG_INFO, "Pathfinder A2: reached obstacle edge.");
			else
				syslog(LOG_INFO, "Pathfinder A2: stopped in front of obstacle.");

			steering.mode = STEERING_ROTATE;
			if (sensors_data.position_x < 0)
				steering.desired_heading = MAX_HEADING;
			else
				steering.desired_heading = -MAX_HEADING;
			steering.max_heading_rate = MAX_HEADING_RATE_NORMAL;
		}

		if (can_resume && fabs(sensors_data.position_x) < 2.0)
		{
			steering.mode = STEERING_ROTATE;
			steering.desired_heading = 0.0;
			steering.max_heading_rate = MAX_HEADING_RATE_NORMAL;

			if (sensors_data.heading > 0.0)
				rotate_dir = -1.0;
			else
				rotate_dir = 1.0;
			pathfinder_a2_mode = PATHFINDER_A2_FINISH;
		}
		break;
	case PATHFINDER_A2_ROTATE_TO_CENTER:
		if (steering.mode == STEERING_STOP)
		{
			can_resume = 1;
			syslog(LOG_INFO, "Pathfinder A2: Rotated to center.");

			double distance_to_course = sensors_data.position_x /
				sin((90.0 - sensors_data.heading) * M_PI / 180.0);
			if (sensors_data.distance > fabs(distance_to_course))
			{
				steering.mode = STEERING_DRIVE_FORWARD;
				sensors_data_reset_odo();
				steering.desired_odo = fabs(distance_to_course) * 2;
				pathfinder_a2_mode = PATHFINDER_A2_DRIVE_FORWARD;
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
				steering.mode = STEERING_ROTATE;
				steering.desired_heading = rotate_dir * MAX_HEADING;
				steering.max_heading_rate = MAX_HEADING_RATE_SEARCH;
				pathfinder_a2_mode = PATHFINDER_A2_SEARCH_EDGE;
			}
		}
		break;
	case PATHFINDER_A2_FINISH:
		if (steering.mode == STEERING_STOP)
		{
			stats_end();
			syslog(LOG_INFO, "Pathfinder A2: elapsed %.2f s, covered %.2f cm.",
				stats.elapsed_time, stats.distance_covered);
			pathfinder_a2_mode = PATHFINDER_A2_NONE;
			mode_switch(MODE_SUPERVISOR);
			if (ipc_raspberry_daemon_attach() < 0)
				syslog(LOG_ERR, "Cannot enter supervisor mode.");
		}
		break;
	}
}
