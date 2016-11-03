/*
 * shared_memory.h
 *
 * Author: Marek Nalepa, ported from C++ to C, original code by Wojciech Michna
 * Purpose: Provide functions to handle files mapping as memory regions.
 */

#ifndef SHARED_MEMORY_H_
#define SHARED_MEMORY_H_

/* Create read-only mapping */
int shared_memory_map_rdonly(const char* file, int* fd, void** ptr, int size);

/* Create read/write mapping */
int shared_memory_map_rdwr(const char* file, int* fd, void** ptr, int size);

/* Remove mapping */
void shared_memory_unmap(int* fd, void** ptr, int size);

#endif /* SHARED_MEMORY_H_ */
