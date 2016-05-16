#include "common.h"
#include "daemon.h"

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

	return 0;
}
