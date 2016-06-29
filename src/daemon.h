#ifndef DAEMON_H_
#define DAEMON_H_

#include "common.h"

void daemon_init(const char* name, int debug, const char* pidfile);
void daemon_destroy(void);

#endif /* DAEMON_H_ */
