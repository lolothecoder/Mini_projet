#include "ch.h"
#include "hal.h"
#include "motors.h"
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <main.h>
#include <motors_lib.h>
#include <TOF.h>
#include <audio_processing.h>
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

static bool broken_loop = false;
static int dist_travelled[2];
static bool Lshape = false;

static THD_WORKING_AREA(waTOF, 1024);
static THD_FUNCTION(TOF, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1)
	{
		if(find_dist(OBSTACLE_DISTANCE))
		{
			chprintf((BaseSequentialStream *)&SD3, "SPOTTED OBSTACLE\r\n\n");

			int motor_pos = right_motor_get_pos();
			int motor_pos_cm = steps_to_dist(motor_pos);

			dist_travelled[0] = motor_pos_cm;
			dist_travelled[1] = 0;

			int right_mot_new_pos = dodge_obstacle();
			quarter_turns(SINGLE_TURN, LEFT_TURN);

			if(broken_loop && Lshape)
			{
				right_motor_set_pos(dist_to_steps(right_mot_new_pos));
			}
			else
			{
				right_motor_set_pos(motor_pos+dist_to_steps(right_mot_new_pos));
			}
			set_speed(MOTOR_SPEED);

			dist_travelled[0] = 0;
			dist_travelled[1] = 0;

			broken_loop = false;
			Lshape = false;
			chprintf((BaseSequentialStream *)&SD3, "CLEARED OBSTACLE\r\n\n");
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

bool multi_dist(uint8_t samples, uint8_t distance)
{
	for(uint8_t i = 0; i < samples; i++){
		if(find_dist(distance)) return true;
	}
	return false;
}

int distance_till_safe(int dist_travelled)
{
	int distance = 0;
	int counter = 0;
	while(counter < NUM_OF_1_ON_16_TURNS && !broken_loop)
	{
		if (find_dist(OBSTACLE_DISTANCE)) { //can play with the number here
			chprintf((BaseSequentialStream *)&SD3, "L SHAPE\r\n\n");
			distance += dodge_obstacle();
			Lshape = true;
			quarter_turns(SINGLE_TURN, LEFT_TURN);
			return distance;
		}
		else{
			if(Lshape || verify_dist(distance, ADVANCE_DIST, dist_travelled)){
				straight_line(ADVANCE_DIST, STRAIGHT);
				distance += ADVANCE_DIST;
				counter = search();
			}
			else{
				broken_loop = true;
				int dist_left = LOOP_DISTANCE - dist_travelled - distance;
				straight_line(dist_left,STRAIGHT);
				quarter_turns(SINGLE_TURN, LEFT_TURN);
				//infinite_stop();
			}
		}
	}
	return distance;
}

int get_closer(int distance)
{
	init_pos_motor();
	if(!find_dist(OBSTACLE_DISTANCE)){
		set_speed(600);
		while(!find_dist(OBSTACLE_DISTANCE) && abs(right_motor_get_pos()) < dist_to_steps(distance)){}
	}
	if(abs(right_motor_get_pos()) >= dist_to_steps(distance)) return dist_to_steps(distance);
	return abs(right_motor_get_pos());
}

uint8_t search(void)
{
	uint8_t counter = 0;
	for(uint8_t i = 0; i < NUM_OF_1_ON_16_TURNS; i++){
		counter++;
		eight_times_two_turns(SINGLE_TURN,RIGHT_TURN, SEARCH_SPEED);
		if(multi_dist(NUM_SAMPLES, OBSTACLE_DISTANCE + ERROR)) break;
	}
	eight_times_two_turns(counter,LEFT_TURN, SEARCH_SPEED);
	return counter;
}

bool object_removed(int distances[2])
{
	if(search() == NUM_OF_1_ON_16_TURNS)
	{
		quarter_turns(SINGLE_TURN, RIGHT_TURN);
		distances[0] -= steps_to_dist(get_closer(distances[0])) + CALC_ERROR;

		if (distances[0] == 0) return true;

		quarter_turns(SINGLE_TURN,LEFT_TURN);
	}
	return false;
}

int dodge_obstacle(void)
{
	int distances[DIST_SIZE] = {0};
	quarter_turns(SINGLE_TURN,LEFT_TURN);

	//chprintf((BaseSequentialStream *)&SD3, "INIT = %d%\r\n\n", motor_pos);

	distances[0] = distance_till_safe(dist_travelled[1]);
	quarter_turns(SINGLE_TURN, RIGHT_TURN);

	if(Lshape || verify_dist(0,ADVANCE_DIST_END,dist_travelled[0]))
	{
		straight_line(ADVANCE_DIST_END, STRAIGHT);
		distances[1] += ADVANCE_DIST_END;

		if(!object_removed(distances))
		{
			distances[1] += distance_till_safe(dist_travelled[0] + ADVANCE_DIST_END);
			quarter_turns(SINGLE_TURN, RIGHT_TURN);
		}
	}
	else{
		broken_loop = true;
		int dist_left = LOOP_DISTANCE - dist_travelled[0];
		straight_line(dist_left,STRAIGHT);
	}

	//chprintf((BaseSequentialStream *)&SD3, "DIST = %d%\r\n\n", distances[0]);
	if(distances[0]!= 0 && !broken_loop) straight_line(distances[0], STRAIGHT);
	if(broken_loop){
		return distances[0];
	}
	else{
		return distances[1];
	}
}

bool verify_dist(int distance, int added_dist, int dist_travelled)
{
	chprintf((BaseSequentialStream *)&SD3, "VERIFY = %d%\r\n\n", dist_travelled +distance + added_dist);

	if(dist_travelled +distance + added_dist < LOOP_DISTANCE+1) return true;
	return false;

}






