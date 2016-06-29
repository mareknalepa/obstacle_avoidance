#ifndef IPC_H_
#define IPC_H_

#include "common.h"

int ipc_init(void);
int ipc_raspberry_daemon_detach(void);
int ipc_raspberry_daemon_attach(void);

#endif /* IPC_H_ */
