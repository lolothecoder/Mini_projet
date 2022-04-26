#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <main.h>
#include <TOF.h>

#define OBSTACLE_DISTANCE 	80
#define ADVANCE_DIST		2
#define DIST_TO_SUB			10
#define ERROR				12

#define FIRST_IMPLEMENTATION

static uint8_t obstacle = 0;
static uint8_t counter = 0;
static uint8_t save_dist = 1;
static uint8_t dist_parc = 0;
static uint8_t dist_to_add = 0;


static THD_WORKING_AREA(waTOF, 256);
static THD_FUNCTION(TOF, arg) {
	systime_t time;
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    while(1){
//    	if (get_status() == 0){
//    		while(1){
//    			chThdSleepMilliseconds(10);
//    		}
//    	}
#ifdef FIRST_IMPLEMENTATION
    	if(obstacle == 0){
    		if(VL53L0X_get_dist_mm() < OBSTACLE_DISTANCE - DIST_TO_SUB){
    			obstacle = 1;
    		} else {
    			obstacle = 3;
    		}
    	}

    	if(obstacle == 1){
    		quarter_turns(1,1);
    		straight_line(ADVANCE_DIST, 1);
    		dist_parc += ADVANCE_DIST;
    		advance_till_safe();
    		save_dist = 2;
    		obstacle = 1;
    		straight_line(ADVANCE_DIST+2);
    		dist_to_add += ADVANCE_DIST+2;
    		advance_till_safe();
    		straight_line(dist_parc, 1);
    		quarter_turns(1,1);
    		obstacle = 3;
    		save_dist = 1;
    		dist_parc = 0;
    	}
#else


#endif
    	chThdSleepMilliseconds(100);
    	//chThdYield();
    }
}


void TOF_start(void){
	chThdCreateStatic(waTOF, sizeof(waTOF), NORMALPRIO+1, TOF, NULL);
}

uint8_t get_obstacle(void){
	return obstacle;
}

void set_obstacle(uint8_t obst){
	obstacle = obst;
}

uint8_t get_dist_to_add(void){
	return dist_to_add;
}

void set_dist_to_add(uint8_t dist){
	dist_to_add = dist;
}

void advance_till_safe(void)
{
	while(obstacle == 1){
		eight_times_two_turns(1,-1);
	    counter ++;
	    if(VL53L0X_get_dist_mm() < OBSTACLE_DISTANCE){
	    	eight_times_two_turns(counter,1);
	    	counter = 0;
	    	straight_line(ADVANCE_DIST);
	    	if(save_dist == 1){
	    		dist_parc += ADVANCE_DIST;
	    	}
	    	if(save_dist == 2){
	    		dist_to_add += ADVANCE_DIST;
	    	}
	    }
	    if (counter > 7){
	    	counter = 0;
	    	eight_times_two_turns(4,1);
	    	obstacle = 2;
	    }
	}

}

void find_appropriate_dist(uint8_t distance, uint8_t margin){
	while(VL53L0X_get_dist_mm() > distance || VL53L0X_get_dist_mm() < distance-margin){
		if(VL53L0X_get_dist_mm() > distance){
			straight_line(1, 1);
		} else if(VL53L0X_get_dist_mm() < distance-margin){
			straight_line(1, -1);
		}
	}
}


