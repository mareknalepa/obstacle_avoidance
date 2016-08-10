#include "common.h"
#include "daemon.h"
#include "scheduler.h"
#include "sensors_data.h"
#include "ipc.h"
#include "modes.h"
#include "motors.h"
#include "supervisor.h"
#include "brake.h"
#include "pathfinder.h"

static void signal_handler(int signum);

int main(int argc, char** argv)
{
	int option = 0;
	int debug = 0;

	/* Parse command line options */
	while ((option = getopt(argc, argv, "d")) != -1)
	{
		switch (option)
		{
		case 'd':
			debug = 1;
			break;
		default:
			break;
		}
	}
	
	/* Set process priority */
	scheduler_init(98);

	/* Init daemon */
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
	mode_register_handler(MODE_PATHFINDER, &pathfinder_action);

	/* Begin infinite loop */
	while (1)
	{
		scheduler_begin_cycle();
		sensors_data_filter();
		mode_action();
		scheduler_end_cycle();
	}

	return 0;
}

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
