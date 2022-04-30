#include "ch.h"
#include "hal.h"
#include "motors.h"
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <main.h>
#include <motors_lib.h>
#include <TOF.h>
#include <stdlib.h>

#define OBSTACLE_DISTANCE 		60
#define ADVANCE_DIST			4
#define ADVANCE_DIST_END		6
#define ERROR					20
#define NUM_SAMPLES				20
#define NUM_OF_1_ON_16_TURNS	8
#define SEARCH_SPEED			400
#define DIST_SIZE				2
#define CALC_ERROR				1

static THD_WORKING_AREA(waTOF, 1024);
static THD_FUNCTION(TOF, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    while(1){
    	if(find_dist(OBSTACLE_DISTANCE)){
    		int motor_pos = right_motor_get_pos();
    		int right_mot_new_pos = dodge_obstacle();
    		quarter_turns(SINGLE_TURN, LEFT_TURN);
    		right_motor_set_pos(motor_pos+dist_to_steps(right_mot_new_pos));
    		set_speed(MOTOR_SPEED);
    	}
    	chThdSleepMilliseconds(100);
    }
}


void TOF_start(void){
	chThdCreateStatic(waTOF, sizeof(waTOF), NORMALPRIO+1, TOF, NULL);
}

bool find_dist(uint8_t distance){
	if(VL53L0X_get_dist_mm() < distance){
		return true;
	} else {
		return false;
	}
}

bool multi_dist(uint8_t samples, uint8_t distance){
	for(uint8_t i = 0; i < samples; i++){
		if(find_dist(distance)) return true;
	}
	return false;
}

int distance_till_safe(void){
	int distance = 0;
	int counter = 0;
	while(counter < NUM_OF_1_ON_16_TURNS){
		if (find_dist(OBSTACLE_DISTANCE)){
			chprintf((BaseSequentialStream *)&SD3, "DONE = %d%\r\n\n", stop);
			distance += dodge_obstacle();
			quarter_turns(SINGLE_TURN, LEFT_TURN);
			return distance;
		}
		else{
			straight_line(ADVANCE_DIST, STRAIGHT);
			distance += ADVANCE_DIST;
			counter = search();
		}
	}
	return distance;
}

int get_closer(int distance){
	init_pos_motor();
	if(!find_dist(OBSTACLE_DISTANCE)){
		set_speed(600);
		while(!find_dist(OBSTACLE_DISTANCE) && abs(right_motor_get_pos()) < dist_to_steps(distance)){}
	}
	if(abs(right_motor_get_pos()) >= dist_to_steps(distance)) return dist_to_steps(distance);
	return abs(right_motor_get_pos());
}

uint8_t search(void){
	uint8_t counter = 0;
	for(uint8_t i = 0; i < NUM_OF_1_ON_16_TURNS; i++){
		counter++;
		eight_times_two_turns(SINGLE_TURN,RIGHT_TURN, SEARCH_SPEED);
		if(multi_dist(NUM_SAMPLES, OBSTACLE_DISTANCE + ERROR)) break;
	}
	eight_times_two_turns(counter,LEFT_TURN, SEARCH_SPEED);
	return counter;
}

bool object_removed(int distances[2]){
	if(search() == NUM_OF_1_ON_16_TURNS){
		quarter_turns(SINGLE_TURN, RIGHT_TURN);
	    distances[0] -= steps_to_dist(get_closer(distances[0])) + CALC_ERROR;
	    if (distances[0] == 0) return true;
	    quarter_turns(SINGLE_TURN,LEFT_TURN);
	}
	return false;
}

int dodge_obstacle(void){
	int distances[DIST_SIZE] = {0};
	quarter_turns(SINGLE_TURN,LEFT_TURN);
	//chprintf((BaseSequentialStream *)&SD3, "INIT = %d%\r\n\n", motor_pos);
	distances[0] = distance_till_safe();
	quarter_turns(SINGLE_TURN, RIGHT_TURN);
	straight_line(ADVANCE_DIST_END, STRAIGHT);
	distances[1] += ADVANCE_DIST_END;
	if(!object_removed(distances)){
		distances[1] += distance_till_safe();
		quarter_turns(SINGLE_TURN, RIGHT_TURN);
	}
	//chprintf((BaseSequentialStream *)&SD3, "DIST = %d%\r\n\n", distances[0]);
	if(distances[0]!= 0) straight_line(distances[0], STRAIGHT);
	//chprintf((BaseSequentialStream *)&SD3, "END = %d%\r\n\n", motor_pos+dist_to_steps(distances[1]));
	return distances[1];
}
