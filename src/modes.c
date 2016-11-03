/*
 * modes.c
 *
 * Author: Marek Nalepa
 * Purpose: Handle application mode state machine.
 */

#include "modes.h"

#include "common.h"

application_mode_t mode = MODE_SUPERVISOR;
application_mode_handler_t mode_handlers[3] = { 0 };

/* Switch to another mode */
void mode_switch(application_mode_t new_mode)
{
	syslog(LOG_INFO, "Switching mode from '%s' to '%s'...",
			mode_to_str(mode), mode_to_str(new_mode));
	mode = new_mode;
}

/* Get human-readable mode name */
const char* mode_to_str(application_mode_t mode)
{
	switch (mode)
	{
	case MODE_SUPERVISOR:
		return "supervisor";
		break;
	case MODE_BRAKE:
		return "brake";
		break;
	case MODE_PATHFINDER:
		return "pathfinder";
		break;
	default:
		return "unknown";
		break;
	}
}

/* Register logic action handler for specific mode */
void mode_register_handler(application_mode_t mode,
		application_mode_handler_t handler)
{
	if (mode < 0 || mode > 2)
		return;

	mode_handlers[mode] = handler;
}

/* Perform current mode's logic action */
void mode_action(void)
{
	mode_handlers[mode]();
}
