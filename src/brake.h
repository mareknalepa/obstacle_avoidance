#ifndef BRAKE_H_
#define BRAKE_H_

#include "common.h"

#define DISTANCE_TRESHOLD	80
#define DISTANCE_STOP		50
#define DISTANCE_DIFF		(DISTANCE_TRESHOLD - DISTANCE_STOP)

void brake_action(void);

#endif /* BRAKE_H_ */
