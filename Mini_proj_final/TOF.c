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

#define OBSTACLE_DISTANCE 		60	//The distance at which the robot can detect an obstacle

#define ADVANCE_DIST			4	//The distance the robot can advance while dodging before
									//checking for the obstacle

#define ADVANCE_DIST_END		6	//Distance the robot travels after dodging one side of the
									//obstacle

#define ERROR					20	//Margin of error of TOF while checking for an obstacle

#define NUM_OF_1_ON_16_TURNS	8	//Numbers of 1/16 turns to do to check if the obstacle
									// has been dodged

#define SEARCH_SPEED			400	//Speed at which the robot turns to look for an obstacle

#define DIST_SIZE				2 	//The amount of distances the robot has to remember

#define AXIS_1					0
#define AXIS_2					1
#define CALC_ERROR				1 	//Needs to be added while computing a distance

//Checks if the robot tries to go further than loop_distance
//defined in audio_processing.c
static bool broken_loop = false;

//Keeps tracks of the distance the robot travelled before dodging an obstacle
static int dist_travelled[DIST_SIZE];

//Verifies if the object to dodge is an L shape obstacle
static bool Lshape = false;

static THD_WORKING_AREA(waTOF, 1024);

//The prototypes are defined here since dodge_obstacle calls distance_till_safe and
//distance_till_safe calls dodge_obstacle
int dodge_obstacle(void);
int distance_till_safe(int dist_travelled);

//Verifies if the obstacle is a certain distance from the robot
bool find_dist(uint8_t distance){
	if(VL53L0X_get_dist_mm() < distance){
		return true;
	} else {
		return false;
	}
}

//Approaches the obstacle if we have gone too far from it
int get_closer(int distance)
{
	init_pos_motor();
	if(!find_dist(OBSTACLE_DISTANCE)){
		set_speed(MOTOR_SPEED);
		while(!find_dist(OBSTACLE_DISTANCE) && abs(right_motor_get_pos()) < dist_to_steps(distance)){}
	}
	if(abs(right_motor_get_pos()) >= dist_to_steps(distance)) return dist_to_steps(distance);
	return abs(right_motor_get_pos());
}

//Verifies if the obstacle is still on the right of the robot
uint8_t search(void)
{
	uint8_t counter = 0;
	for(uint8_t i = 0; i < NUM_OF_1_ON_16_TURNS; i++){
		eight_times_two_turns(SINGLE_TURN,RIGHT_TURN, SEARCH_SPEED);
		counter++;
		if(find_dist(OBSTACLE_DISTANCE + ERROR)){
			//chprintf((BaseSequentialStream *)&SD3, "counter right = %d%\r\n\n", counter);
			break;
		}
	}
	eight_times_two_turns(counter,LEFT_TURN, SEARCH_SPEED);
	//chprintf((BaseSequentialStream *)&SD3, "counter left = %d%\r\n\n", counter);
	return counter;
}

//Checks if the obstacle has been removed. If so we go back in the loop
bool object_removed(int distances[DIST_SIZE])
{
	if(search() == NUM_OF_1_ON_16_TURNS)
	{
		quarter_turns(SINGLE_TURN, RIGHT_TURN);
		distances[AXIS_1] -= steps_to_dist(get_closer(distances[AXIS_1])) + CALC_ERROR;

		if (distances[AXIS_1] == 0) return true;

		quarter_turns(SINGLE_TURN,LEFT_TURN);
	}
	return false;
}

//checks if we are still in the loop
bool verify_dist(int distance, uint8_t added_dist, int dist_travelled)
{
	if(dist_travelled +distance + added_dist < get_loop_distance ()+CALC_ERROR) return true;
	return false;
}

int dodge_obstacle(void)
{
	int distances[DIST_SIZE] = {0};
	quarter_turns(SINGLE_TURN,LEFT_TURN);

	distances[AXIS_1] = distance_till_safe(dist_travelled[AXIS_2]); //Dodges first side of the obstacle
	quarter_turns(SINGLE_TURN, RIGHT_TURN);

	if(Lshape || verify_dist(0,ADVANCE_DIST_END,dist_travelled[AXIS_1])) //Verifies if we can advance safely without
																	//breaking the loop or hitting a wall
	{
		straight_line(ADVANCE_DIST_END, STRAIGHT);
		distances[AXIS_2] += ADVANCE_DIST_END;

		if(!object_removed(distances)) //checks that the object is still there
		{
			distances[AXIS_2] += distance_till_safe(dist_travelled[AXIS_1] + ADVANCE_DIST_END); //dodges second side
			quarter_turns(SINGLE_TURN, RIGHT_TURN);
		}
	}
	else{
		broken_loop = true;
		uint8_t dist_left = get_loop_distance () - dist_travelled[AXIS_1];
		straight_line(dist_left,STRAIGHT);
	}

	//restores distance travelled to dodge the first side
	if(distances[AXIS_1]!= 0 && !broken_loop) straight_line(distances[AXIS_1], STRAIGHT);
	if(broken_loop){
		return distances[AXIS_1];
	}
	else{
		return distances[AXIS_2];
	}
}

//Checks how far the robot has to go to clear the obstacle
//if the robot is able to do a full 180 without seeing an object
//then he has cleared the obstacle
int distance_till_safe(int dist_travelled)
{
	uint8_t distance = 0; //the distance the robot has to travel
	uint8_t counter = 0; //the amount of turns done by the robot
	while(counter < NUM_OF_1_ON_16_TURNS && !broken_loop)
	{
		if (find_dist(OBSTACLE_DISTANCE)) { //checks if the object is L shaped
			chprintf((BaseSequentialStream *)&SD3, "L SHAPE\r\n\n");
			distance += dodge_obstacle(); //dodges L shape object
			Lshape = true;
			quarter_turns(SINGLE_TURN, LEFT_TURN);
			return distance;
		}
		else{
			//Verifies if we can advance without breaking loop_distance or
			//if L shape is activated it ignores that check
			if(Lshape || verify_dist(distance, ADVANCE_DIST, dist_travelled)){
				straight_line(ADVANCE_DIST, STRAIGHT);
				distance += ADVANCE_DIST;
				counter = search();
			}
			else{
				//If we try to break the LOOP_DISTANCE(main.h) we go back in the loop
				broken_loop = true;
				uint8_t dist_left = get_loop_distance () - dist_travelled - distance;
				straight_line(dist_left,STRAIGHT);
				quarter_turns(SINGLE_TURN, LEFT_TURN);
			}
		}
	}
	return distance;
}

static THD_FUNCTION(TOF, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1)
	{
		if(find_dist(OBSTACLE_DISTANCE)) //checks if there is an obstacle
		{

			chprintf((BaseSequentialStream *)&SD3, "SPOTTED OBSTACLE\r\n\n");
			//saves the motor postion at the beggining of the thread
			int motor_pos = right_motor_get_pos();
			int motor_pos_cm = steps_to_dist(motor_pos);

			//saves the distances travelled in x and y
			dist_travelled[AXIS_1] = motor_pos_cm;
			dist_travelled[AXIS_2] = 0;
			int right_mot_new_pos = dodge_obstacle();	//dodges obstacle
			quarter_turns(SINGLE_TURN, LEFT_TURN);		//Turns left to continue the loop after the obstacle

			//Restores motor position
			if(broken_loop)
			{
				right_motor_set_pos(dist_to_steps(right_mot_new_pos));
			}
			else
			{
				right_motor_set_pos(motor_pos+dist_to_steps(right_mot_new_pos));
			}

			set_speed(MOTOR_SPEED);

			//resets the distances travelled
			dist_travelled[AXIS_1] = 0;
			dist_travelled[AXIS_2] = 0;

			//resets essential bools
			broken_loop = false;
			Lshape = false;
			chprintf((BaseSequentialStream *)&SD3, "CLEARED OBSTACLE\r\n\n");
		}
		chThdSleepMilliseconds(100);
	}
}

//Starts the thread that will deal with obstacles
void TOF_start(void){
	chThdCreateStatic(waTOF, sizeof(waTOF), NORMALPRIO, TOF, NULL);
}

