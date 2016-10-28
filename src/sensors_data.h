#ifndef SENSORS_DATA_H_
#define SENSORS_DATA_H_

typedef struct {
    double distance;
    double heading;
    double heading_rate;
    double global_odo;
    double odo;
    double position_x;
    double position_y;
} sensors_data_t;

extern sensors_data_t sensors_data;

int sensors_data_init(void);
void sensors_data_destroy(void);
void sensors_data_filter(void);
void sensors_data_reset_odo(void);
void sensors_data_reset_coordinates(void);

#endif /* SENSORS_DATA_H_ */
