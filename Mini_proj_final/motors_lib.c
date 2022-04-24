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

//static int direction = 0;

void init_pos_motor(void)
{
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

void straight_line(uint8_t distance)
{
	init_pos_motor();
	left_motor_set_speed(MOTOR_SPEED);
	right_motor_set_speed(MOTOR_SPEED);
	while(right_motor_get_pos() < distance* NSTEP_ONE_TURN / WHEEL_PERIMETER){}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void quarter_turns(uint8_t num_of_quarter_turns)
{
	init_pos_motor();
	left_motor_set_speed(-MOTOR_SPEED);
	right_motor_set_speed(MOTOR_SPEED);
	while(right_motor_get_pos() < num_of_quarter_turns*PERIMETER_EPUCK/4* NSTEP_ONE_TURN / WHEEL_PERIMETER){}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void straight_then_turn(uint8_t distance)
{
	straight_line(distance);
	quarter_turns(1);
}

static THD_WORKING_AREA(waLoop, 256);
static THD_FUNCTION(Loop, arg) {
	systime_t time;
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	time = chVTGetSystemTime();
    	straight_then_turn(20);
        chThdSleepUntilWindowed(time, time + MS2ST(5000));
    }
}

void loop_start(void){
	chThdCreateStatic(waLoop, sizeof(waLoop), NORMALPRIO, Loop, NULL);
}

