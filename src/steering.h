/*
 * steering.h
 *
 * Author: Marek Nalepa
 * Purpose: Controls motors to achieve smooth movement of vehicle.
 */

#ifndef STEERING_H_
#define STEERING_H_

#include "sensors_data.h"

/* Steering possible states */
typedef enum {
    STEERING_STOP = 0,
    STEERING_DRIVE_FORWARD,
    STEERING_ROTATE,
} steering_mode_t;

/* Desired effects of movement */
typedef struct {
    steering_mode_t mode;
    double desired_odo;
    double desired_heading;
    double max_heading_rate;
} steering_t;

extern steering_t steering;

/* Steering action in application cycle */
void steering_cycle(void);

/* Drives forward by a predetermined distance */
inline void steering_drive(double length)
{
    steering.mode = STEERING_DRIVE_FORWARD;
    steering.desired_odo = sensors_data.odo + length;
}

/* Rotate vehicle to specified heading with specified rate */
inline void steering_rotate(double heading, double rate)
{
    steering.mode = STEERING_ROTATE;
    steering.desired_heading = heading;
    steering.max_heading_rate = rate;
}

#endif /* STEERING_H_ */
