#include "stats.h"

#include "common.h"
#include "sensors_data.h"
#include <sys/time.h>

stats_t stats;

static struct timeval t1, t2;
static double initial_odo;

void stats_start(void)
{
	initial_odo = sensors_data.global_odo;
	gettimeofday(&t1, NULL);
}

void stats_end(void)
{
	gettimeofday(&t2, NULL);
	double t1_s = t1.tv_sec + t1.tv_usec / 1000000.0;
	double t2_s = t2.tv_sec + t2.tv_usec / 1000000.0;
	stats.elapsed_time = t2_s - t1_s;
	stats.distance_covered = sensors_data.global_odo - initial_odo;
}
