#ifndef STEERING_H_
#define STEERING_H_

#include "sensors_data.h"

typedef enum {
    STEERING_STOP = 0,
    STEERING_DRIVE_FORWARD,
    STEERING_ROTATE,
} steering_mode_t;

typedef struct {
    steering_mode_t mode;
    double desired_odo;
    double desired_heading;
    double max_heading_rate;
} steering_t;

extern steering_t steering;

void steering_cycle(void);

inline void steering_drive(double length)
{
    sensors_data_reset_odo();
    steering.mode = STEERING_DRIVE_FORWARD;
    steering.desired_odo = length;
}

inline void steering_rotate(double heading, double rate)
{
    steering.mode = STEERING_ROTATE;
    steering.desired_heading = heading;
    steering.max_heading_rate = rate;
}

#endif /* STEERING_H_ */
