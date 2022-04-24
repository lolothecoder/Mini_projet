#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>

#define OBSTACLE_DISTANCE 	70

static uint8_t obstacle = 0;
static uint8_t counter = 0;
static uint8_t save_dist = 1;
static uint8_t dist_parc = 0;

static THD_WORKING_AREA(waTOF, 256);
static THD_FUNCTION(TOF, arg) {
	systime_t time;
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	time = chVTGetSystemTime();
    	if(obstacle == 0){
    		if(VL53L0X_get_dist_mm() < OBSTACLE_DISTANCE){
    			obstacle = 1;
    		} else {
    			obstacle = 3;
    		}
    	}

    	if(obstacle == 1){
    		quarter_turns(1,1);
    		straight_line(2);
    		advance_till_safe();
    		save_dist = 0;
    		obstacle = 1;
    		straight_line(4);
    		advance_till_safe();
    		straight_line(dist_parc);
    		quarter_turns(1,1);
    		obstacle = 3;

    	}
    	chThdSleepMilliseconds(500);
    }


    		//sixteen_turns(1,-1);
    		//obstacle = 0;
}
    	//chprintf((BaseSequentialStream *)&SD3, "distance = %d%\r\n\n", VL53L0X_get_dist_mm());
    	//chThdSleepUntilWindowed(time, time + 300);

void TOF_start(void){
	chThdCreateStatic(waTOF, sizeof(waTOF), NORMALPRIO+1, TOF, NULL);
}

uint8_t get_obstacle(void){
	return obstacle;
}

uint8_t set_obstacle(uint8_t obst){
	obstacle = obst;
}

void advance_till_safe(void)
{
	while(obstacle == 1){
		eight_times_two_turns(1,-1);
	    counter ++;
	    if(VL53L0X_get_dist_mm() < OBSTACLE_DISTANCE){
	    	eight_times_two_turns(counter,1);
	    	counter = 0;
	    	straight_line(2);
	    	if(save_dist == 1){
	    		dist_parc += 2;
	    	}
	    }
	    if (counter > 7){
	    	counter = 0;
	    	eight_times_two_turns(4,1);
	    	obstacle = 2;
	    }
	}

}


