#include "ch.h"
#include "hal.h"
#include "motors.h"

#define MOTOR_SPEED   		600 // []
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]
#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

static int direction = 0;

void init_pos_motor(void)
{
	right_motor_set_pos(0);
}

void straight_line(uint8_t distance)
{
	while(right_motor_get_pos() < distance* NSTEP_ONE_TURN / WHEEL_PERIMETER){
		left_motor_set_speed(MOTOR_SPEED);
		right_motor_set_speed(MOTOR_SPEED);
	}
}

void quarter_turns(uint8_t num_of_quarter_turns)
{
	while(right_motor_get_pos() < num_of_quarter_turns*PERIMETER_EPUCK/4* NSTEP_ONE_TURN / WHEEL_PERIMETER){
		left_motor_set_speed(-MOTOR_SPEED);
		right_motor_set_speed(MOTOR_SPEED);
	}
}

void square(uint8_t distance)
{
	if(direction == 1){
		straight_line(distance);
	    right_motor_set_pos(0);
	    direction = -1;
	    }
	else{
		quarter_turns(1);
	    right_motor_set_pos(0);
	    direction = 1;
	}
}


