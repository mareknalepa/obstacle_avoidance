#ifndef MODES_H_
#define MODES_H_

#include "common.h"

typedef enum {
	MODE_SUPERVISOR = 0,
	MODE_BRAKE,
	MODE_PATHFINDER,
} application_mode_t;

typedef void (*application_mode_handler_t)(void);

extern application_mode_t mode;
extern application_mode_handler_t mode_handlers[3];

void mode_switch(application_mode_t new_mode);
const char* mode_to_str(application_mode_t mode);
void mode_register_handler(application_mode_t mode,
		application_mode_handler_t handler);
void mode_action(void);

#endif /* MODES_H_ */
