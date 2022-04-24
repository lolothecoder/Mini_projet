#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>

#define OBSTACLE_DISTANCE 	80

static uint8_t obstacle = 0;

static THD_WORKING_AREA(waTOF, 256);
static THD_FUNCTION(TOF, arg) {
	//systime_t time;
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	if(obstacle != 1){
    		if(VL53L0X_get_dist_mm() < OBSTACLE_DISTANCE){
    			obstacle = 1;
    		} else {
    			obstacle = 2;
    		}
    	}

    	if(obstacle == 1){
    		quarter_turns(1);
    		straight_line(2);
    		obstacle = 0;
    	}
    	//chprintf((BaseSequentialStream *)&SD3, "distance = %d%\r\n\n", VL53L0X_get_dist_mm());
    	chThdSleepMilliseconds(20);
    }
}

void TOF_start(void){
	chThdCreateStatic(waTOF, sizeof(waTOF), NORMALPRIO+1, TOF, NULL);
}

uint8_t get_obstacle(){
	return obstacle;
}

uint8_t set_obstacle(uint8_t obst){
	obstacle = obst;
}
