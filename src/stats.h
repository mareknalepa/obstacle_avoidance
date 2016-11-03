#ifndef STATS_H_
#define STATS_H_

typedef struct {
    double elapsed_time;
    double distance_covered;
    int heading_changes;
    int forward_rides;
} stats_t;

extern stats_t stats;

void stats_start(void);
void stats_end(void);

#endif /* STATS_H_ */
