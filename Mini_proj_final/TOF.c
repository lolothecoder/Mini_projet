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

#define CALC_ERROR				1 	//Needs to be added while computing a distance

#define THRESHOLD_TOF			14

//Alignment stuff might remove later
#define ALIGNMENT				10
#define SAMPLES					20

//Checks if the robot tries to go further than LOOP_DISTANCE
//defined in main.h
static bool broken_loop = false;

//Keeps tracks of the distance the robot travelled before dodging an obstacle
static int dist_travelled[DIST_SIZE];

//Verifies if the object to dodge is an L shape obstacle
static bool Lshape = false;

static THD_WORKING_AREA(waTOF, 1024);
static THD_FUNCTION(TOF, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1)
	{
		if(check_dist(OBSTACLE_DISTANCE)) //checks if there is an obstacle
		{

			chprintf((BaseSequentialStream *)&SD3, "SPOTTED OBSTACLE\r\n\n");
			//saves the motor postion at the beggining of the thread
			int motor_pos = right_motor_get_pos();
			int motor_pos_cm = steps_to_dist(motor_pos);

			//saves the distances travelled in x and y
			dist_travelled[0] = motor_pos_cm;
			dist_travelled[1] = 0;

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

			set_speed(get_current_speed ());

			//resets the distances travelled
			dist_travelled[0] = 0;
			dist_travelled[1] = 0;

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

//Verifies if the obstacle is a certain distance from the robot
bool find_dist(uint8_t distance){
	if(VL53L0X_get_dist_mm() < distance){
		return true;
	} else {
		return false;
	}
}

//Function to attempt to reduce TOF error by taking multiple measurements
//and taking the average.
int multi_dist(void)
{
	int distance_fin = 0;
	for(uint8_t i = 0; i < SAMPLES; i++){
		distance_fin += VL53L0X_get_dist_mm();
	}
	return distance_fin/SAMPLES;
}

bool check_dist(uint16_t distance){
	uint8_t counter = 0;
	bool answer = false;
	for(uint8_t i = 0; i < SAMPLES; i++){
		if(VL53L0X_get_dist_mm() < distance) counter++;
	}
	if (counter > THRESHOLD_TOF){
		answer = true;
	}
	return answer;
}
//Checks how far the robot has to go to clear the obstacle
//if the robot is able to do a full 180 without seeing an object
//then he has cleared the obstacle
int distance_till_safe(int dist_travelled)
{
	int distance = 0; //the distance the robot has to travel
	int counter = 0; //the amount of turns done by the robot
	while(counter < NUM_OF_1_ON_16_TURNS && !broken_loop)
	{
		if (check_dist(OBSTACLE_DISTANCE)) { //checks if the object is L shaped
			chprintf((BaseSequentialStream *)&SD3, "L SHAPE\r\n\n");
			distance += dodge_obstacle(); //dodges L shape object
			Lshape = true;
			quarter_turns(SINGLE_TURN, LEFT_TURN);
			return distance;
		}
		else{
			//Verifies if we can advance without breaking LOOP_DISTANCE(main.h) or
			//if L shape is activated it ignores that check
			if(Lshape || verify_dist(distance, ADVANCE_DIST, dist_travelled)){
				straight_line(ADVANCE_DIST, STRAIGHT);
				distance += ADVANCE_DIST;
				counter = search();
			}
			else{
				//If we try to break the LOOP_DISTANCE(main.h) we go back in the loop
				broken_loop = true;
				int dist_left = get_loop_distance () - dist_travelled - distance;
				straight_line(dist_left,STRAIGHT);
				quarter_turns(SINGLE_TURN, LEFT_TURN);
			}
		}
	}
	return distance;
}

//Approaches the obstacle if we have gone too far from it
int get_closer(int distance)
{
	init_pos_motor();
	if(!check_dist(OBSTACLE_DISTANCE)){
		set_speed(get_current_speed ());
		while(!check_dist(OBSTACLE_DISTANCE) && abs(right_motor_get_pos()) < dist_to_steps(distance)){}
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
		if(check_dist(OBSTACLE_DISTANCE + ERROR)){
			chprintf((BaseSequentialStream *)&SD3, "counter right = %d%\r\n\n", counter);
			break;
		}
	}
	eight_times_two_turns(counter,LEFT_TURN, SEARCH_SPEED);
	chprintf((BaseSequentialStream *)&SD3, "counter left = %d%\r\n\n", counter);
	return counter;
}

//Checks if the obstacle has been removed. If so we go back in the loop
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

	distances[0] = distance_till_safe(dist_travelled[1]); //Dodges first side of the obstacle
	quarter_turns(SINGLE_TURN, RIGHT_TURN);

	if(Lshape || verify_dist(0,ADVANCE_DIST_END,dist_travelled[0])) //Verifies if we can advance safely without
																	//breaking the loop or hitting a wall
	{
		straight_line(ADVANCE_DIST_END, STRAIGHT);
		distances[1] += ADVANCE_DIST_END;

		if(!object_removed(distances)) //checks that the object is still there
		{
			distances[1] += distance_till_safe(dist_travelled[0] + ADVANCE_DIST_END); //dodges second side
			quarter_turns(SINGLE_TURN, RIGHT_TURN);
		}
	}
	else{
		broken_loop = true;
		int dist_left = get_loop_distance () - dist_travelled[0];
		straight_line(dist_left,STRAIGHT);
	}

	//restores distance travelled to dodge the first side
	if(distances[0]!= 0 && !broken_loop) straight_line(distances[0], STRAIGHT);
	if(broken_loop){
		return distances[0];
	}
	else{
		return distances[1];
	}
}

//checks if we are still in the loop
bool verify_dist(int distance, int added_dist, int dist_travelled)
{
	//chprintf((BaseSequentialStream *)&SD3, "VERIFY = %d%\r\n\n", dist_travelled +distance + added_dist);

	if(dist_travelled +distance + added_dist < get_loop_distance ()+CALC_ERROR) return true;
	return false;

}

// Testing if a 90 deg alignment of the robot to the obstacle can be added
void align(void){
	uint16_t last_distance = 0;
	uint16_t last_distance_r = 10000;
	uint16_t last_distance_l = 10000;
	int8_t direction = 0;

	hundreed_turn(10, RIGHT_TURN);
	last_distance_r = multi_dist();
	hundreed_turn(20, LEFT_TURN);
	last_distance_l = multi_dist();
	hundreed_turn(10, RIGHT_TURN);

	if (last_distance_r < last_distance_l) direction = RIGHT_TURN;
	else direction = LEFT_TURN;


	while(1){
		last_distance = multi_dist();
		hundreed_turn(1, direction);
		if(multi_dist() < OBSTACLE_DISTANCE + 50 && multi_dist() > last_distance ){
			hundreed_turn(1, -direction);
			hundreed_turn(10, direction);
			if(multi_dist() < OBSTACLE_DISTANCE + 50 && multi_dist() > last_distance ){
				hundreed_turn(10, -direction);
				return;
			}
		}
	}
}



