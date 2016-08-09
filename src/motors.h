#ifndef MOTORS_H_
#define MOTORS_H_

int motors_init(void);
void motors_destroy(void);
void motors_read(int* left, int* right);
void motors_write(int left, int right);

#endif /* MOTORS_H_ */
