/*
 * daemon.h
 *
 * Author: Marek Nalepa
 * Purpose: Run application as system daemon.
 */

#ifndef DAEMON_H_
#define DAEMON_H_

/* Initialize daemon, open system log detach from console */
void daemon_init(const char* name, int debug, const char* pidfile);

/* Shutdown daemon */
void daemon_destroy(void);

#endif /* DAEMON_H_ */
