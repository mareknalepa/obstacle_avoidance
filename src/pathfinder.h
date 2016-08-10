#ifndef PATHFINDER_H_
#define PATHFINDER_H_

typedef enum {
    PATHFINDER_NONE,
    PATHFINDER_LOOKUP_LEFT_ROTATE,
    PATHFINDER_LOOKUP_LEFT_RETURN,
    PATHFINDER_LOOKUP_RIGHT_ROTATE,
    PATHFINDER_LOOKUP_RIGHT_RETURN,
    PATHFINDER_SET_HEADING,
    PATHFINDER_DRIVE,
} pathfinder_mode_t;

extern pathfinder_mode_t pathfinder_mode;

void pathfinder_action(void);

#endif /* PATHFINDER_H_ */
