#include "daemon.h"

void daemon_init(const char* name, int debug, const char* pidfile)
{
	pid_t pid, sid;
	uid_t uid;
	int descriptor;
	int flags;
	char pid_buffer[16] = { 0 };
	struct flock file_lock;

	/* Open system log */
	if (debug)
		openlog(name, LOG_PID | LOG_PERROR, LOG_DAEMON);
	else
		openlog(name, LOG_PID, LOG_DAEMON);

	syslog(LOG_INFO, "Starting %s daemon...", name);

	/* Check if run by root */
	uid = getuid();
	if (uid != 0)
	{
		syslog(LOG_ERR, "Application must be run by root.");
		exit(1);
	}

	/* If debug mode is selected, daemon initialization is now completed */
	if (debug)
		return;

	/* Fork off the first time */
	pid = fork();
	if (pid < 0)
		exit(1);

	/* Success: Let the parent terminate */
	if (pid > 0)
	{
		syslog(LOG_INFO, "Created first child process with PID: %d\n", pid);
		exit(0);
	}

	/* Create a new SID for the child process */
	sid = setsid();
	if (sid < 0)
		exit(1);

	/* Fork off for the second time*/
	pid = fork();
	if (pid < 0)
		exit(1);

	/* Success: Let the parent terminate */
	if (pid > 0)
	{
		syslog(LOG_INFO, "Created second child process with PID: %d\n", pid);
		exit(0);
	}

	/* Change the file mode mask */
	umask(0);

	/* Change the current working directory */
	if ((chdir("/")) < 0)
		exit(1);

	/* Close out file descriptors */
	descriptor = sysconf(_SC_OPEN_MAX);
	while (descriptor)
		close(descriptor--);

	/* Reopen standard descriptors */
	stdin = fopen("/dev/null", "r");
	stdout = fopen("/dev/null", "w+");
	stderr = fopen("/dev/null", "w+");

	syslog(LOG_INFO, "Creating PID file '%s'...", pidfile);

	/* Attempt to create PID file */
	descriptor = open(pidfile, O_RDWR | O_CREAT, S_IWUSR | S_IRUSR);
	if (descriptor < 0)
	{
		syslog(LOG_ERR, "Cannot create PID file.");
		exit(1);
	}

	/* Set close-on-exit flag */
	flags = fcntl(descriptor, F_GETFD);
	if (flags < 0)
	{
		syslog(LOG_ERR, "Cannot create PID file.");
		exit(1);
	}
	flags |= FD_CLOEXEC;
	if (fcntl(descriptor, F_SETFD, flags) < 0)
	{
		syslog(LOG_ERR, "Cannot create PID file.");
		exit(1);
	}

	/* Attempt to lock PID file */
	file_lock.l_type = F_WRLCK;
	file_lock.l_whence = SEEK_SET;
	file_lock.l_start = 0;
	file_lock.l_len = 0;
	if (fcntl(descriptor, F_SETLK, &file_lock) < 0)
	{
		if (errno == EACCES || errno == EAGAIN)
		{
			syslog(LOG_ERR, "Cannot lock PID file, probably in use "
					"by another instance.");
			exit(1);
		} else {
			syslog(LOG_ERR, "Cannot lock PID file, unexpected error occurred.");
			exit(1);
		}
	}

	/* Attempt to truncate PID file */
	if (ftruncate(descriptor, 0) < 0)
	{
		syslog(LOG_ERR, "Cannot create PID file.");
		exit(1);
	}

	/* Write PID of current process to PID file */
	snprintf(pid_buffer, sizeof(pid_buffer), "%ld\n", (long) getpid());
	if (write(descriptor, pid_buffer, strlen(pid_buffer))
			!= strlen(pid_buffer))
	{
		syslog(LOG_ERR, "Cannot create PID file.");
		exit(1);
	}

	syslog(LOG_INFO, "Created PID file '%s'.", pidfile);
}
