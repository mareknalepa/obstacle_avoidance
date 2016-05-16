#ifndef DAEMON_H_
#define DAEMON_H_

#include "common.h"

void daemon_init(const char* name, int debug, const char* pidfile);

#endif /* DAEMON_H_ */
