/*
 * scheduler.c
 *
 * Author: Marek Nalepa
 * Purpose: Setup application as real-time task and handle cycles.
 */

#include "scheduler.h"

#include "common.h"
#include "constants.h"
#include <sched.h>
#include <time.h>

static struct timespec scheduler_timer;

/* Setup scheduler */
void scheduler_init(int priority)
{
	struct sched_param sch_param;
	sch_param.__sched_priority = priority;
	if (sched_setscheduler(0, SCHED_FIFO, &sch_param) < 0)
	{
		syslog(LOG_ERR, "Cannot set priority!");
		exit(1);
	}

	clock_gettime(CLOCK_MONOTONIC, &scheduler_timer);
}

/* Start cycle */
void scheduler_begin_cycle(void)
{
	scheduler_timer.tv_nsec += SCHEDULER_CYCLE_INTERVAL;
	if (scheduler_timer.tv_nsec >= 1000000000)
	{
		scheduler_timer.tv_nsec -= 1000000000;
		++scheduler_timer.tv_sec;
	}
}

/* Stop cycle, wait before next cycle */
void scheduler_end_cycle(void)
{
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &scheduler_timer, 0);
}
