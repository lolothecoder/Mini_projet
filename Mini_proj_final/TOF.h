/*
 * TOF.h
 *
 *  Created on: 24 avr. 2022
 *      Author: lolon
 */

#ifndef TOF_H_
#define TOF_H_

void TOF_start(void);
uint8_t get_obstacle(void);
void set_obstacle(uint8_t obst);
void advance_till_safe(void);
uint8_t get_dist_to_add(void);
void set_dist_to_add(uint8_t dist);
#endif /* TOF_H_ */
