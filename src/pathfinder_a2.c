#include "pathfinder_a2.h"

#include "common.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "motors.h"
#include "brake.h"
#include <math.h>

typedef enum {
    PATHFINDER_A2_NONE,
	PATHFINDER_A2_SEARCH_EDGE_LEFT,
	PATHFINDER_A2_SEARCH_EDGE_RIGHT,
	PATHFINDER_A2_DRIVE_FORWARD,
	PATHFINDER_A2_FINISH,
} pathfinder_a2_mode_t;

pathfinder_a2_mode_t pathfinder_a2_mode = PATHFINDER_A2_NONE;

#define MOTORS_SPEED 50
#define MAX_HEADING 60
#define VEHICLE_WIDTH 20

static int prev_distances[2];
static double prev_headings[2];
static int prev_index = 0;
static int samples_number = 0;
static int desired_distance = 0;

#define HEADING_OFFSET 5
#define PATHFINDER_A1_DIST 30

void pathfinder_a2_action(void)
{
	switch (pathfinder_a2_mode)
	{
	case PATHFINDER_A2_NONE:
		sensors_data.heading = 0;
		sensors_data_reset_odo();
		prev_distances[0] = 0;
		prev_distances[1] = 0;
		prev_headings[0] = 0.0;
		prev_headings[1] = 0.0;
		prev_index = 0;
		samples_number = 0;
		motors_write(-MOTORS_SPEED, MOTORS_SPEED);
		pathfinder_a2_mode = PATHFINDER_A2_SEARCH_EDGE_LEFT;
		syslog(LOG_INFO, "Pathfinder A2: searching edge of obstacle (left)...");
		break;
	case PATHFINDER_A2_SEARCH_EDGE_LEFT: 
		if (sensors_data.heading <= -MAX_HEADING)
		{
			motors_write(0, 0);
			syslog(LOG_INFO, "Pathfinder A2: reached max angle.");
			motors_write(MOTORS_SPEED, -MOTORS_SPEED);
			pathfinder_a2_mode = PATHFINDER_A2_SEARCH_EDGE_RIGHT;
			syslog(LOG_INFO, "Pathfinder A2: searching edge of obstacle "
				"(right)...");
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

			if (fabs(predicted_distance - sensors_data.dist->distance)
				> tolerance)
			{
				desired_distance = prev_distances[prev_index];
				motors_write(100, 100);
				pathfinder_a2_mode = PATHFINDER_A2_DRIVE_FORWARD;
				syslog(LOG_INFO, "Pathfinder A2: found edge at "
					"direction %.1f.", sensors_data.heading);
			}
		}

		prev_index = !prev_index;
		prev_distances[prev_index] = sensors_data.dist->distance;
		prev_headings[prev_index] = sensors_data.heading;
		++samples_number;
		break;
	case PATHFINDER_A2_SEARCH_EDGE_RIGHT:
		if (sensors_data.heading <= MAX_HEADING)
		{
			motors_write(0, 0);
			syslog(LOG_INFO, "Pathfinder A2: reached max angle.");
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

			if (fabs(predicted_distance - sensors_data.dist->distance)
				> tolerance)
			{
				motors_write(0, 0);
				pathfinder_a2_mode = PATHFINDER_A2_FINISH;
				syslog(LOG_INFO, "Pathfinder A2: found edge at "
					"direction %.1f.", sensors_data.heading);
			}
		}

		prev_index = !prev_index;
		prev_distances[prev_index] = sensors_data.dist->distance;
		prev_headings[prev_index] = sensors_data.heading;
		++samples_number;
		break;
	case PATHFINDER_A2_DRIVE_FORWARD:
		if (sensors_data.odo >= desired_distance)
		{
			motors_write(0, 0);
			pathfinder_a2_mode = PATHFINDER_A2_FINISH;
			syslog(LOG_INFO, "Pathfinder A2: reached obstacle edge.");
		}
		break;
	case PATHFINDER_A2_FINISH:
		motors_write(0, 0);
		pathfinder_a2_mode = PATHFINDER_A2_NONE;
		mode_switch(MODE_SUPERVISOR);
		if (ipc_raspberry_daemon_attach() < 0)
			syslog(LOG_ERR, "Cannot enter supervisor mode.");
		break;
	}
}
