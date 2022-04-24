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
uint8_t set_obstacle(uint8_t obst);
void advance_till_safe(void);
#endif /* TOF_H_ */
