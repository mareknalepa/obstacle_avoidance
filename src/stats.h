/*
 * stats.h
 *
 * Author: Marek Nalepa
 * Purpose: Provide statistic data of obstacle avoiding process.
 */

#ifndef STATS_H_
#define STATS_H_

/* Data presented to user */
typedef struct {
    double elapsed_time;
    double distance_covered;
    int heading_changes;
    int forward_rides;
} stats_t;

extern stats_t stats;

/* Mark obstacle avoiding process as started */
void stats_start(void);

/* Mark obstacle avoiding process as finished and sum up */
void stats_end(void);

#endif /* STATS_H_ */
