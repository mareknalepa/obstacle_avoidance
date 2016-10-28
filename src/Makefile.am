AM_CFLAGS = -Wall -pedantic
AM_LDFLAGS = -lm

bin_PROGRAMS = obstacle_avoidance
obstacle_avoidance_SOURCES = \
	brake.c \
	daemon.c \
	ipc.c \
	main.c \
	modes.c \
	motors.c \
	pathfinder_a1.c \
	pathfinder_a2.c \
	scheduler.c \
	sensors_data.c \
	shared_memory.c \
	stats.c \
	steering.c \
	supervisor.c

noinst_HEADERS = \
	brake.h \
	common.h \
	daemon.h \
	ipc.h \
	modes.h \
	motors.h \
	pathfinder_a1.h \
	pathfinder_a2.h \
	scheduler.h \
	sensors_data.h \
	shared_memory.h \
	stats.h \
	steering.h \
	supervisor.h
