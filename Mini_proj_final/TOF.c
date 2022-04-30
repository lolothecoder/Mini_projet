#include "ch.h"
#include "hal.h"
#include "motors.h"
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <main.h>
#include <motors_lib.h>
#include <TOF.h>

#define OBSTACLE_DISTANCE 		60
#define ADVANCE_DIST			4
#define ERROR					10
#define NUM_SAMPLES				10
#define NUM_OF_1_ON_16_TURNS	8
#define SEARCH_SPEED			400
#define MAX_SIZE				6
#define FINAL_INDEX_I			2
#define FINAL_ITERATION_I		FINAL_INDEX_I-1


//#define FIRST_IMPLEMENTATION

static uint8_t distances[MAX_SIZE] = {0};

static THD_WORKING_AREA(waTOF, 256);
static THD_FUNCTION(TOF, arg) {
	systime_t time;
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    while(1){

    	if(find_dist(OBSTACLE_DISTANCE)){
    		int motor_pos = right_motor_get_pos();
    		chprintf((BaseSequentialStream *)&SD3, "INIT = %d%\r\n\n", motor_pos);
    		uint8_t start_dist = 0;
    		quarter_turns(SINGLE_TURN,LEFT_TURN);
    		for(uint8_t i = 0; i < FINAL_INDEX_I; i++){
    			distances[i] = distance_till_safe() +start_dist;
    			quarter_turns(SINGLE_TURN, RIGHT_TURN);
    			if(i < FINAL_ITERATION_I){
    				straight_line(ADVANCE_DIST + 2, STRAIGHT);
    				start_dist = ADVANCE_DIST+2;
    				if(search() == NUM_OF_1_ON_16_TURNS){
    					quarter_turns(SINGLE_TURN, RIGHT_TURN);
    					distances[i] -= steps_to_dist(get_closer(distances[i]));
    					distances[i+1] += start_dist;
    					break;
    				}
    			}
    		}
    		if(distances[0]!= 0) straight_line(distances[0], STRAIGHT);
    		quarter_turns(SINGLE_TURN, LEFT_TURN);

    		right_motor_set_pos(motor_pos+dist_to_steps(distances[1]));
    		reset_distances();
    		set_speed(MOTOR_SPEED);
    		chprintf((BaseSequentialStream *)&SD3, "END = %d%\r\n\n", motor_pos+dist_to_steps(distances[1]));
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

uint16_t distance_till_safe(void){
	uint8_t distance = 0;
	uint8_t counter = 0;
	while(counter < NUM_OF_1_ON_16_TURNS){
		straight_line(ADVANCE_DIST,STRAIGHT);
		distance += ADVANCE_DIST;
		counter = search();
	}
	return distance;
}

void reset_distances(void){
	for(uint8_t i = 0; i < MAX_SIZE; i++){
		distances[i] = 0;
	}
}

int get_closer(uint8_t distance){
	init_pos_motor();
	set_speed(600);
	while(!find_dist(OBSTACLE_DISTANCE) && right_motor_get_pos() < dist_to_steps(distance)){}
	set_speed(0);
	if(right_motor_get_pos() > dist_to_steps(distance)) return dist_to_steps(distance);
	return right_motor_get_pos();
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
