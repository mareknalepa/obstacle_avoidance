#ifndef MOTORS_H_
#define MOTORS_H_

typedef struct {
	int left;
	int right;
} motors_t;

extern motors_t* motors;

int motors_init(void);
void motors_destroy(void);
void motors_write(int left, int right);

#endif /* MOTORS_H_ */
