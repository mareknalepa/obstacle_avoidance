#ifndef SCHEDULER_H_
#define SCHEDULER_H_

void scheduler_init(int priority);
void scheduler_begin_cycle(void);
void scheduler_end_cycle(void);

#endif /* SCHEDULER_H_ */
