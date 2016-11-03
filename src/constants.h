/*
 * constants.h
 *
 * Author: Marek Nalepa
 * Purpose: Store various constant values.
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

/* Vehicle properties */
#define VEHICLE_WIDTH                   20
#define VEHICLE_LENGTH                  20
#define LONGITUDINAL_CLEARANCE          30
#define TRANSVERSE_CLEARANCE            10

#define SCHEDULER_CYCLE_INTERVAL        100000000;
#define SCHEDULER_PRIORITY              89
#define ODO_FILTER_SMOOTHNESS           0.3

/* Steering PID values */
#define DRIVE_MAX_OUT                   100.0
#define DRIVE_P                         6.0

#define HEADING_RATE_P                  4.0
#define HEADING_RATE_I                  8.0
#define HEADING_RATE_D                  5.0
#define HEADING_RATE_SUM_MAX            100.0
#define HEADING_RATE_MAX_OUT            100.0

#define HEADING_P                       0.3
#define HEADING_I                       0.001
#define HEADING_D                       0.004
#define HEADING_SUM_MAX                 200.0
#define HEADING_MAX_OUT_DEFAULT         5.0

/* Supervisor and brake tresholds */
#define DISTANCE_BRAKE                  80
#define DISTANCE_STOP                   60
#define DISTANCE_DIFF                   (DISTANCE_BRAKE - DISTANCE_STOP)

/* Pathfinder constraints */
#define MAX_HEADING                     80
#define HEADING_RATE_TURN               7.0
#define HEADING_RATE_LOOKUP             3.5

/* Pathfinder A1 tune values */
#define PATHFINDER_A1_SAMPLES           56
#define PATHFINDER_A1_MAX_DISTANCE      200
#define PATHFINDER_A1_DRIVING_LENGTH    60
#define PATHFINDER_A1_PREFER_A          -0.00007
#define PATHFINDER_A1_PREFER_B          0.005
#define PATHFINDER_A1_PREFER_C          1.2
#define PATHFINDER_A1_PREFER_MIN_DIST   40
#define PATHFINDER_A1_PREFER_ALT        0.5

/* Pathfinder A2 tune values */
#define PATHFINDER_A2_TOLERANCE         0.1

#endif /* CONSTANTS_H */
