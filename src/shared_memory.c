#include "shared_memory.h"

#include "common.h"
#include <sys/mman.h>

int shared_memory_map_rdonly(const char* file, int* fd, void** ptr, int size)
{
	*fd = open(file, O_RDONLY | O_CLOEXEC);
	if (*fd < 0)
		return -1;

	*ptr = mmap(0, size, PROT_READ, MAP_SHARED, *fd, 0);
	if (*ptr == MAP_FAILED)
	{
		close(*fd);
		*fd = -1;
		return -1;
	}

	return 0;
}

int shared_memory_map_rdwr(const char* file, int* fd, void** ptr, int size)
{
	*fd = open(file, O_RDWR | O_CREAT | O_CLOEXEC, (mode_t) 0777);
	if (*fd < 0)
		return -1;

	*ptr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, *fd, 0);
	if (*ptr == MAP_FAILED)
	{
		close(*fd);
		*fd = -1;
		return -1;
	}

	return 0;
}

void shared_memory_unmap(int* fd, void** ptr, int size)
{
	if (*fd < 0)
		return;

	munmap(*ptr, size);
	*ptr = 0;
	close(*fd);
	*fd = -1;
}
