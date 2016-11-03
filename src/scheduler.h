/*
 * scheduler.h
 *
 * Author: Marek Nalepa
 * Purpose: Setup application as real-time task and handle cycles.
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

/* Setup scheduler */
void scheduler_init(int priority);

/* Start cycle */
void scheduler_begin_cycle(void);

/* Stop cycle, wait before next cycle */
void scheduler_end_cycle(void);

#endif /* SCHEDULER_H_ */
