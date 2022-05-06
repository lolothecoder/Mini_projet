/*
 * TOF.h
 *
 *  Created on: 24 avr. 2022
 *      Author: lolon
 */

#ifndef TOF_H_
#define TOF_H_

void TOF_start(void);

void reset_distances(void);

bool find_dist(uint8_t distance);

int distance_till_safe(int dist_travelled);

bool multi_dist(uint8_t samples, uint8_t distance);

int get_closer(int distance);

uint8_t search(void);

bool object_removed(int distances[2]);

int dodge_obstacle(void);

bool verify_dist(int distance, int added_dist, int dist_travelled);

#endif /* TOF_H_ */





