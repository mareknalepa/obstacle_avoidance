/*
 * ipc.h
 *
 * Author: Marek Nalepa
 * Purpose: Handle IPC (inter-process communication) with main vehicle daemon.
 */

#ifndef IPC_H_
#define IPC_H_

/* Initialize IPC subsystem */
int ipc_init(void);

/* Tell raspberry_daemon not to control motors */
int ipc_raspberry_daemon_detach(void);

/* Tell raspberry_daemon to take control again */
int ipc_raspberry_daemon_attach(void);

#endif /* IPC_H_ */
