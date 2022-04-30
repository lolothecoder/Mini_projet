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
uint16_t distance_till_safe(void);
bool multi_dist(uint8_t samples, uint8_t distance);
int get_closer(uint8_t distance);
uint8_t search(void);
bool object_removed(uint8_t index);
#endif /* TOF_H_ */
