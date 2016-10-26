#ifndef STEERING_H_
#define STEERING_H_

typedef enum {
    STEERING_STOP = 0,
    STEERING_BRAKE,
    STEERING_DRIVE_FORWARD,
    STEERING_ROTATE,
} steering_mode_t;

typedef struct {
    steering_mode_t mode;
    double desired_space;
    double desired_heading;
    double desired_odo;
} steering_t;

extern steering_t steering;

void steering_cycle(void);

#endif /* STEERING_H_ */
