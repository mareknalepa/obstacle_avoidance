/*
 * sensors_data.h
 *
 * Author: Marek Nalepa
 * Purpose: Provide fresh sensors and derived data.
 */

#ifndef SENSORS_DATA_H_
#define SENSORS_DATA_H_

/* Data derived from sensors data */
typedef struct {
    double distance;
    double heading;
    double heading_rate;
    double odo;
    double position_x;
    double position_y;
} sensors_data_t;

extern sensors_data_t sensors_data;

/* Init sensors subsytem */
int sensors_data_init(void);

/* Shutdown sensors system */
void sensors_data_destroy(void);

/* Filter data and calculate derived values */
void sensors_data_filter(void);

/* Reset coordinates system to begin in current position */
void sensors_data_reset_coordinates(void);

#endif /* SENSORS_DATA_H_ */
