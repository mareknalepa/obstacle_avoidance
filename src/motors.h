/*
 * motors.h
 *
 * Author: Marek Nalepa, ported from C++ to C, original code by Wojciech Michna
 * Purpose: Control vehicle's motors.
 */

#ifndef MOTORS_H_
#define MOTORS_H_

typedef struct {
	int left;
	int right;
} motors_t;

/* Current motors state */
extern motors_t* motors;

/* Initialize motors subsystem */
int motors_init(void);

/* Shutdown motors subsystem */
void motors_destroy(void);

/* Set motors' control values */
void motors_write(int left, int right);

#endif /* MOTORS_H_ */
