#include "ipc.h"

#include "common.h"

#define PIDOF_COMMAND "pidof raspberry_deamon"
static pid_t ipc_raspberry_daemon_pid;

int ipc_init(void)
{
	FILE* fp = popen(PIDOF_COMMAND, "re");
	if (!fp)
	{
		syslog(LOG_ERR, "Cannot get raspberry_daemon PID.");
		return -1;
	}
	if (fscanf(fp, "%d", &ipc_raspberry_daemon_pid) != 1)
	{
		syslog(LOG_ERR, "Cannot get raspberry_daemon PID.");
		pclose(fp);
		return -1;
	}
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

int ipc_raspberry_daemon_detach(void)
{
	if (ipc_raspberry_daemon_pid == 0)
		return -1;

	if (kill(ipc_raspberry_daemon_pid, SIGUSR1) < 0)
	{
		syslog(LOG_ERR, "Error sending SIGUSR1 to process %d: '%s'.",
			 ipc_raspberry_daemon_pid, strerror(errno));
		return -1;
	}
	
	return 0;
}

int ipc_raspberry_daemon_attach(void)
{
	if (ipc_raspberry_daemon_pid == 0)
		return -1;

	if (kill(ipc_raspberry_daemon_pid, SIGUSR2) < 0)
	{
		syslog(LOG_ERR, "Error sending SIGUSR2 to process %d: '%s'.",
			 ipc_raspberry_daemon_pid, strerror(errno));
		return -1;
	}
	
	return 0;
}
