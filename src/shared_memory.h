#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

int shared_memory_map_rdonly(const char* file, int* fd, void** ptr, int size);
int shared_memory_map_rdwr(const char* file, int* fd, void** ptr, int size);
void shared_memory_unmap(int* fd, void** ptr, int size);

#endif /* SHARED_MEMORY_H */
