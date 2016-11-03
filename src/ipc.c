/*
 * ipc.c
 *
 * Author: Marek Nalepa
 * Purpose: Handle IPC (inter-process communication) with main vehicle daemon.
 */

#include "ipc.h"

#include "common.h"

#define PIDOF_COMMAND "pidof raspberry_deamon"
static pid_t ipc_raspberry_daemon_pid;

/* Initialize IPC subsystem */
int ipc_init(void)
{
	/* Run 'pidof' system command */
	FILE* fp = popen(PIDOF_COMMAND, "re");
	if (!fp)
	{
		syslog(LOG_ERR, "Cannot get raspberry_daemon PID.");
		return -1;
	}
	/* Read 'pidof' output to get raspberry_daemon PID */
	if (fscanf(fp, "%d", &ipc_raspberry_daemon_pid) != 1)
	{
		syslog(LOG_ERR, "Cannot get raspberry_daemon PID.");
		pclose(fp);
		return -1;
	}
	/* Sanity check */
	if (ipc_raspberry_daemon_pid == 0)
	{
		syslog(LOG_ERR, "Invalid raspberry_daemon PID read.");
		pclose(fp);
		return -1;
	}

	syslog(LOG_INFO, "Raspberry daemon PID: %d", ipc_raspberry_daemon_pid);
	pclose(fp);

	return 0;
}

/* Tell raspberry_daemon not to control motors */
int ipc_raspberry_daemon_detach(void)
{
	if (ipc_raspberry_daemon_pid == 0)
		return -1;

	/* Send SIGUSR1 signal to raspberry_daemon */
	if (kill(ipc_raspberry_daemon_pid, SIGUSR1) < 0)
	{
		syslog(LOG_ERR, "Error sending SIGUSR1 to process %d: '%s'.",
			 ipc_raspberry_daemon_pid, strerror(errno));
		return -1;
	}

	return 0;
}

/* Tell raspberry_daemon to take control again */
int ipc_raspberry_daemon_attach(void)
{
	if (ipc_raspberry_daemon_pid == 0)
		return -1;

	/* Send SIGUSR2 signal to raspberry_daemon */
	if (kill(ipc_raspberry_daemon_pid, SIGUSR2) < 0)
	{
		syslog(LOG_ERR, "Error sending SIGUSR2 to process %d: '%s'.",
			 ipc_raspberry_daemon_pid, strerror(errno));
		return -1;
	}

	return 0;
}
