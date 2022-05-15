#include "ch.h"
#include "hal.h"
#include "motors.h"

//#include <TOF.h>
#include <chprintf.h>
#include <communications.h>
#include <motors_lib.h>
#include <stdlib.h>
#include <selector.h>
#include <audio_processing.h>


#define MAX_SELECTOR_VALUE	15
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]
#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define ALIGN_SPEED			200
#define DELTA_SPEED 		60


struct motor_speed {
	int16_t left_speed;
	int16_t right_speed;
};

struct motor_speed motors;

void init_pos_motor(void)
{
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

void straight_line(uint8_t distance, int dir)
{
	/*
	init_pos_motor();
	while(abs(right_motor_get_pos()) < distance* NSTEP_ONE_TURN / WHEEL_PERIMETER){
		left_motor_set_speed(dir * MOTOR_SPEED);
		right_motor_set_speed(dir * MOTOR_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	motors.right_speed = 0;
	*/

	init_pos_motor();
	left_motor_set_speed(dir * MOTOR_SPEED);
	motors.left_speed = dir * MOTOR_SPEED;
	right_motor_set_speed(dir * MOTOR_SPEED);
	motors.right_speed = dir * MOTOR_SPEED;
	while(abs(right_motor_get_pos()) < distance* NSTEP_ONE_TURN / WHEEL_PERIMETER){
		left_motor_set_speed(dir * MOTOR_SPEED);
		right_motor_set_speed(dir * MOTOR_SPEED);
	}
	left_motor_set_speed(0);
	motors.left_speed = 0;
	right_motor_set_speed(0);
	motors.right_speed = 0;
}

void quarter_turns(uint8_t num_of_quarter_turns, int dir)
{
	init_pos_motor();
	if (dir == 1){
		while(right_motor_get_pos() < num_of_quarter_turns*PERIMETER_EPUCK/4* NSTEP_ONE_TURN / WHEEL_PERIMETER){
			left_motor_set_speed(-dir * MOTOR_SPEED);
			right_motor_set_speed(dir * MOTOR_SPEED);
		}
	}else
	{
		while(left_motor_get_pos() < num_of_quarter_turns*PERIMETER_EPUCK/4* NSTEP_ONE_TURN / WHEEL_PERIMETER){
			left_motor_set_speed(-dir * MOTOR_SPEED);
			right_motor_set_speed(dir * MOTOR_SPEED);
		}
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void eight_times_two_turns(uint8_t num_of_sixteen_turns, int dir, uint16_t speed)
{
	init_pos_motor();
	if (dir == 1){
		while(right_motor_get_pos() < num_of_sixteen_turns*PERIMETER_EPUCK/16* NSTEP_ONE_TURN / WHEEL_PERIMETER){
			left_motor_set_speed(-dir * speed);
			right_motor_set_speed(dir * speed);
		}
	}else{
		while(left_motor_get_pos() < num_of_sixteen_turns*PERIMETER_EPUCK/16* NSTEP_ONE_TURN / WHEEL_PERIMETER){
			left_motor_set_speed(-dir * speed);
			right_motor_set_speed(dir * speed);
		}
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void straight_then_turn(uint8_t distance)
{
	straight_line(distance, STRAIGHT);
	quarter_turns(SINGLE_TURN, LEFT_TURN);
}

void set_speed(int speed){
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
}

void stop (void)
{
	left_motor_set_speed (0);
	right_motor_set_speed (0);
}

void infinite_stop(void){
	stop();
	while(1){}
}

void go (void)
{
	left_motor_set_speed (MOTOR_SPEED);
	right_motor_set_speed (MOTOR_SPEED);
	motors.left_speed = MOTOR_SPEED;
	motors.right_speed = MOTOR_SPEED;
}

int dist_to_steps(int distance){
	return distance* NSTEP_ONE_TURN / WHEEL_PERIMETER;
}

int steps_to_dist(int steps){
	return steps*  WHEEL_PERIMETER / NSTEP_ONE_TURN ;
}

int conditional_advance(uint8_t distance, uint8_t dir, bool continue_advance){
	init_pos_motor();
	left_motor_set_speed(dir * MOTOR_SPEED);
	right_motor_set_speed(dir * MOTOR_SPEED);
	while(abs(right_motor_get_pos()) < distance* NSTEP_ONE_TURN / WHEEL_PERIMETER && continue_advance){}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	return abs(right_motor_get_pos());
}

void hundreed_turn(uint8_t num_of_hundreed_turns, int dir)
{
	init_pos_motor();
	if (dir == 1){
		while(right_motor_get_pos() < num_of_hundreed_turns*PERIMETER_EPUCK/300* NSTEP_ONE_TURN / WHEEL_PERIMETER){
			left_motor_set_speed(-dir * ALIGN_SPEED);
			right_motor_set_speed(dir * ALIGN_SPEED);
		}
	}else
	{
		while(left_motor_get_pos() < num_of_hundreed_turns*PERIMETER_EPUCK/300* NSTEP_ONE_TURN / WHEEL_PERIMETER){
			left_motor_set_speed(-dir * ALIGN_SPEED);
			right_motor_set_speed(dir * ALIGN_SPEED);
		}
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}









