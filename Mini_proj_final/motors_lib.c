#include "ch.h"
#include "hal.h"
#include "motors.h"

#include <TOF.h>
#include <chprintf.h>

#include <motors_lib.h>


#define MOTOR_SPEED   		600 // []
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]
#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

//static int direction = 0;

void init_pos_motor(void)
{
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

void straight_line(uint8_t distance, int dir)
{
	init_pos_motor();
	left_motor_set_speed(dir * MOTOR_SPEED);
	right_motor_set_speed(dir * MOTOR_SPEED);
	while(abs(right_motor_get_pos()) < distance* NSTEP_ONE_TURN / WHEEL_PERIMETER){
		//chprintf((BaseSequentialStream *)&SD3, "MOTORS_POS = %d% \r\n\n", right_motor_get_pos());
		if(get_dist_to_add() != 0 && get_obstacle() == 3){
			right_motor_set_pos(right_motor_get_pos() + get_dist_to_add()* NSTEP_ONE_TURN / WHEEL_PERIMETER);
			set_dist_to_add(0);
		}
		if (get_obstacle() == 3){
			left_motor_set_speed(dir * MOTOR_SPEED);
			right_motor_set_speed(dir * MOTOR_SPEED);
			set_obstacle(0);
		}


	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void quarter_turns(uint8_t num_of_quarter_turns, int dir)
{
	init_pos_motor();
	left_motor_set_speed(-dir * MOTOR_SPEED);
	right_motor_set_speed(dir * MOTOR_SPEED);
	if (dir == 1){
		while(right_motor_get_pos() < num_of_quarter_turns*PERIMETER_EPUCK/4* NSTEP_ONE_TURN / WHEEL_PERIMETER){}
	}else
	{
		while(left_motor_get_pos() < num_of_quarter_turns*PERIMETER_EPUCK/4* NSTEP_ONE_TURN / WHEEL_PERIMETER){}
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void eight_times_two_turns(uint8_t num_of_sixteen_turns, int dir)
{
	init_pos_motor();
	left_motor_set_speed(-dir * MOTOR_SPEED);
	right_motor_set_speed(dir * MOTOR_SPEED);
	if (dir == 1){
		while(right_motor_get_pos() < num_of_sixteen_turns*PERIMETER_EPUCK/16* NSTEP_ONE_TURN / WHEEL_PERIMETER){}
	}else{
		while(left_motor_get_pos() < num_of_sixteen_turns*PERIMETER_EPUCK/16* NSTEP_ONE_TURN / WHEEL_PERIMETER){}
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void straight_then_turn(uint8_t distance)
{
	straight_line(distance, 1);
	quarter_turns(1, 1);
}


/*
static THD_WORKING_AREA(waLoop, 256);
static THD_FUNCTION(Loop, arg) {
	//systime_t time;
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	//time = chVTGetSystemTime();
    	//chprintf((BaseSequentialStream *)&SD3, "BEGIN THREAD");
    	straight_then_turn(20);
        //chThdSleepUntilWindowed(time, time + MS2ST(5000));
    }
}

void loop_start(void){
	chThdCreateStatic(waLoop, sizeof(waLoop), NORMALPRIO, Loop, NULL);
}
*/

void stop (void)
{
	left_motor_set_speed (0);
	right_motor_set_speed (0);
}

void go (void)
{
	left_motor_set_speed (MOTOR_SPEED);
	right_motor_set_speed (MOTOR_SPEED);
}
