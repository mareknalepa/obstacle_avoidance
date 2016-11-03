/*
 * main.c
 *
 * Author: Marek Nalepa
 * Purpose: Main application source file with entry point.
 */

#include "common.h"
#include "constants.h"
#include "daemon.h"
#include "scheduler.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "motors.h"
#include "supervisor.h"
#include "brake.h"
#include "pathfinder_a1.h"
#include "pathfinder_a2.h"
#include "steering.h"

/* Handler for external signals (e.g. SIGTERM) to shutdown cleanly */
static void signal_handler(int signum);

/* Application entry point */
int main(int argc, char** argv)
{
	int option = 0;
	int option_index = 0;
	int debug = 0;
	int algorithm = 1;

	/* Parse command line options */
	struct option long_options[] = {
		{"debug",		no_argument,		0,	'd'},
		{"algorithm",	required_argument,	0,	'a'},
		{0,				0,					0,	0},
	};
	opterr = 0;

	while ((option = getopt_long(argc, argv, "a:d", long_options,
								 &option_index)) != -1)
	{
		switch (option)
		{
		case 'a':
			algorithm = atoi(optarg);
			if (algorithm == 0 || algorithm < 1 || algorithm > 2)
			{
				fprintf(stderr, "Invalid argument.\n");
				return -1;
			}
			break;
		case 'd':
			debug = 1;
			break;
		default:
			break;
		}
	}

	/* Set process priority */
	scheduler_init(SCHEDULER_PRIORITY);

	/* Init daemon, daemon mode only if user didn't choose debug mode */
	daemon_init("obstacle_avoidance", debug, "/var/run/obstacle_avoidance.pid");

	/* Register signal handlers */
    signal(SIGINT, &signal_handler);
    signal(SIGTERM, &signal_handler);

	/* Init sensors subsystem */
	if (sensors_data_init() < 0)
		return 1;

	/* Init inter-process communication subsystem */
	if (ipc_init() < 0)
		return 1;

	/* Init motors driver */
	if (motors_init() < 0)
		return 1;

	/* Register application modes actions */
	mode_register_handler(MODE_SUPERVISOR, &supervisor_action);
	mode_register_handler(MODE_BRAKE, &brake_action);

	/* Register pathfinder algorithm basen on user's choice */
	switch (algorithm)
	{
	case 1:
		mode_register_handler(MODE_PATHFINDER, &pathfinder_a1_action);
		break;
	case 2:
		mode_register_handler(MODE_PATHFINDER, &pathfinder_a2_action);
		break;
	default:
		mode_register_handler(MODE_PATHFINDER, &pathfinder_a1_action);
		break;
	}

	syslog(LOG_INFO, "Selected obstacle avoidance algorithm: A%d.", algorithm);

	/* Begin infinite loop */
	while (1)
	{
		/* Start cycle time measure */
		scheduler_begin_cycle();

		/* Obtain fresh sensors data, filter it and make calculations */
		sensors_data_filter();

		/* Perform logic actions on data */
		mode_action();

		/* Update motors with new steering data */
		steering_cycle();

		/* Stop cycle time measure, wait until full cycle */
		scheduler_end_cycle();
	}

	return 0;
}

/* Handler for external signals (e.g. SIGTERM) to shutdown cleanly */
void signal_handler(int signum)
{
	switch (signum)
    {
    case SIGINT:
	case SIGTERM:
		motors_write(0, 0);
		motors_destroy();
		ipc_raspberry_daemon_attach();
		daemon_destroy();
		break;
    }
}
