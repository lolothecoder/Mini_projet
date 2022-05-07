/*
 * TOF.h
 *
 *  Created on: 24 avr. 2022
 *      Author: lolon
 */

#ifndef TOF_H_
#define TOF_H_

//Starts the thread that will deal with obstacles
void TOF_start(void);

//Verifies if the obstacle is a certain distance from the robot
bool find_dist(uint8_t distance);

//Checks how far the robot has to go to clear the obstacle
//if the robot is able to do a full 180 without seeing an object
//then he has cleared the obstacle
int distance_till_safe(int dist_travelled);

//Approaches the obstacle if we have gone too far from it
int get_closer(int distance);

//Searches for an obstacle on the right
uint8_t search(void);

//Checks if the obstacle has been removed
bool object_removed(int distances[2]);

int dodge_obstacle(void);

//Verify that we are not breaking LOOP_DISTANCE(main.h)
bool verify_dist(int distance, int added_dist, int dist_travelled);

void align(void);

#endif /* TOF_H_ */





