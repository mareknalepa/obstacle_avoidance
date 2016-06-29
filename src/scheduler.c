#include "scheduler.h"

#include <sched.h>
#include <time.h>

#define SCHEDULER_CYCLE_INTERVAL 50000000L;

static struct timespec scheduler_timer;

void scheduler_init(int priority)
{
	struct sched_param sch_param;
	sch_param.__sched_priority = priority;
	if (sched_setscheduler(0, SCHED_FIFO, &sch_param) < 0)
	{
		syslog(LOG_DEBUG, "Cannot set priority!");
		exit(1);
	}
	
	clock_gettime(CLOCK_MONOTONIC, &scheduler_timer);
}

void scheduler_begin_cycle(void)
{
	scheduler_timer.tv_nsec += SCHEDULER_CYCLE_INTERVAL;
	if (scheduler_timer.tv_nsec >= 1000000000)
	{
		scheduler_timer.tv_nsec -= 1000000000;
		++scheduler_timer.tv_sec;
	}
}

void scheduler_end_cycle(void)
{
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &scheduler_timer, 0);
}
