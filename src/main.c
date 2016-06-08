#include "common.h"
#include "daemon.h"
#include "sensors_data.h"
#include "modes.h"
#include "supervisor.h"
#include "brake.h"
#include "pathfinder.h"

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

	/* Init daemon */
	daemon_init("obstacle_avoidance", debug, "/var/run/obstacle_avoidance.pid");

	/* Init sensors subsystem */
	if (sensors_data_init() < 0)
		return 1;

	/* Register application modes actions */
	mode_register_handler(MODE_SUPERVISOR, &supervisor_action);
	mode_register_handler(MODE_BRAKE, &brake_action);
	mode_register_handler(MODE_PATHFINDER, &pathfinder_action);

	/* Begin infinite loop */
	while (1)
	{
		if (sensors_data_read() < 0)
			break;
		mode_action();
	}

	return 0;
}
