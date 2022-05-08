#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>

#include <motors_lib.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <TOF.h>

//static int status = 0;
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

//int get_status(){
//	return status;
//}
int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    VL53L0X_start();
    serial_start();
    usb_start();
    motors_init();
    TOF_start();
    mic_start(&processAudioData);
    selector_start ();

    chThdSetPriority(NORMALPRIO-1);

    while (1)
    {
    	straight_then_turn(LOOP_DISTANCE);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

