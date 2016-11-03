/*
 * modes.h
 *
 * Author: Marek Nalepa
 * Purpose: Handle application mode state machine.
 */

#ifndef MODES_H_
#define MODES_H_

/* Possible modes */
typedef enum {
	MODE_SUPERVISOR = 0,
	MODE_BRAKE,
	MODE_PATHFINDER,
} application_mode_t;

/* Mode logic action handler type */
typedef void (*application_mode_handler_t)(void);

extern application_mode_t mode;
extern application_mode_handler_t mode_handlers[3];

/* Switch to another mode */
void mode_switch(application_mode_t new_mode);

/* Get human-readable mode name */
const char* mode_to_str(application_mode_t mode);

/* Register logic action handler for specific mode */
void mode_register_handler(application_mode_t mode,
		application_mode_handler_t handler);

/* Perform current mode's logic action */
void mode_action(void);

#endif /* MODES_H_ */
