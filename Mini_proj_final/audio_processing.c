#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors_lib.h>
#include <TOF.h>
#include "motors.h"
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>


/*
 * Enum used to identify the different orientations that the robot
 * is able to make depending on where the sound  coming from ; used
 * within the functions "determin_sound_state" and "move_to_sound"
 */
typedef enum {
	DEG_0 = 0,
	DEG_45_RIGHT,
	DEG_90_RIGHT,
	DEG_135_RIGHT,
	DEG_180,
	DEG_45_LEFT,
	DEG_90_LEFT,
	DEG_135_LEFT,
	DONT_MOVE //Extra case that shouldn't ever be triggered
} SOUND_ORIENTATION_t;

#define FFT_SIZE 	1024

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

#define HALF_SECOND 		SystemCoreClock/32
#define DEG_45_TURN			2
#define DEG_90_TURN			1
#define DEG_180_TURN		2
#define ONE_TURN			4
#define GO					1
#define STOP				0
#define MIN_VALUE_THRESHOLD	10000 
#define TWO_TURNS			8
#define MIN_VALUE_THRESHOLD	10000
#define EPUCK_DIAMETER		73 //value in mm
#define SOUND_SPEED			343 //value in m/s
#define SOUND_DISTANCE	10 //value in cm

#define SAMPLE_SIZE				5 //Amount of delta x values required to be collected
								   //withinh the function "determine_sound_origin"

#define MIN_DELTA_X_THRESHOLD	10 /*If the delta x value is smaller than this threshold
								   * then the sound is considered to be perpendicular to
								   * the microphone axis in question and thus shouldn't
								   * be affect w
								   */

//we don't analyze before this index to not use resources for nothing
#define MIN_FREQ		10

#define FREQ_250		16	//250Hz
#define FREQ_296		19	//296Hz
#define FREQ_350		23	//350Hz
#define FREQ_406		26	//406Hz
#define FREQ_900		58  //900Hz
#define FREQ_1150		74	//1150Hz
#define FREQ_1400		91  //1400Hz

//we don't analyze after this index to not use resources for nothing
#define MAX_FREQ		100

//Lower and upper bounds used within the sound_remote function
#define FREQ_250_L			(FREQ_250-1)
#define FREQ_250_H			(FREQ_250+1)
#define FREQ_296_L			(FREQ_296-1)
#define FREQ_296_H			(FREQ_296+1)
#define FREQ_350_L			(FREQ_350-1)
#define FREQ_350_H			(FREQ_350+1)
#define FREQ_406_L			(FREQ_406-1)
#define FREQ_406_H			(FREQ_406+1)
#define FREQ_1400_L			(FREQ_1400 -1)
#define FREQ_1400_H			(FREQ_1400 +1)
#define FREQ_900_L			(FREQ_900 -1)
#define FREQ_900_H			(FREQ_900 +1)
#define FREQ_1150_L			(FREQ_1150 -1)
#define FREQ_1150_H			(FREQ_1150 +1)



//Arrays containing delta x samples
static float tab_x_FB[SAMPLE_SIZE];
static float tab_x_LR[SAMPLE_SIZE];

/*
 * Variables containing the averages of the delta_x values
 * that allow us to determine where the sound is coming
 * from
 */
static float x_FB_avg = 0;
static float x_LR_avg = 0;


/*
 * Variable that allows to control whether we've collected enough
 * delta_x samples to compute the average
 */
static int8_t counter_x = 0;

//Gives us the current frequency that's being detected
static int16_t max_norm_index = -1;

//Static variable to know if the robot is moving or not
static int8_t moving = 1;

/*
 * Static variables used within the "processAudioData" function and
 * the "fill_arrays" function to properly fill the arrays of each
 * microphone
 */
static uint16_t nb_samples = 0;
static uint8_t mustSend = 0;

static thread_t *waitThd;
static THD_WORKING_AREA(waWaitThd, 128);
static THD_FUNCTION(WaitThd, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	chSysLock();
    	palTogglePad(GPIOB, GPIOB_LED_BODY);
    	delay(SystemCoreClock);
    	palTogglePad(GPIOB, GPIOB_LED_BODY);
    	delay(HALF_SECOND);
    	palTogglePad(GPIOB, GPIOB_LED_BODY);
    	delay(HALF_SECOND);
    	palTogglePad(GPIOB, GPIOB_LED_BODY);
    	delay(HALF_SECOND);
    	palTogglePad(GPIOB, GPIOB_LED_BODY);
    	delay(HALF_SECOND);
    	palTogglePad(GPIOB, GPIOB_LED_BODY);
    	chSysUnlock();
    	chThdExit(0);
    	waitThd = NULL;
    	chThdYield();
    }
}

int8_t get_moving (void)
{
	return moving;
}

void set_moving (int8_t new_moving)
{
	moving = new_moving;
}

/*
 * Simple Delay Function
 * param : n = number of cycles
 */

void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}

/*
 * Makes body led blink 4 times at certain frequency
 */

void blink_body (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (HALF_SECOND);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (HALF_SECOND);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (HALF_SECOND);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (HALF_SECOND);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (HALF_SECOND);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (HALF_SECOND);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (HALF_SECOND);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
}

/*
 * Function that determines the argument of the complex value
 * corresponding to the current max_norm_index of one of the
 * microphone buffer sent in as a parameter
 *
 * param : data_dft = complex input array of a particular
 * 					  microphone
 * return : argument of the complex value within data_dft
 * 			corresponding to the index
 *
 */

float determine_argument (float* data_dft)
{
	float ratio = (float)(data_dft[2*max_norm_index+1]/data_dft[2*max_norm_index]);

	//REMEMBER TO EXPLAIN THE EXTRA ARCTAN
	ratio = (ratio - (float)((ratio*ratio*ratio)/3));

	//Approximation of arctan
	return (ratio - (float)((ratio*ratio*ratio)/3));
}

/*
 * Function that determines the sound orientation state for the switch
 * within the change orientation function
 *
 * return : the case corresponding to where the sound is coming from
 * 			corresponding to one the fields of the enum
 */

SOUND_ORIENTATION_t determine_sound_state (void)
{
	if (abs(x_LR_avg) < MIN_DELTA_X_THRESHOLD)
	{
		if (x_FB_avg > 0) return DEG_0; else return DEG_180;
	}

	if (abs(x_FB_avg) < MIN_DELTA_X_THRESHOLD)
	{
		if (x_LR_avg < 0) return DEG_90_RIGHT; else return DEG_90_LEFT;
	}

	if ((abs(x_FB_avg) > MIN_DELTA_X_THRESHOLD) &&
		(abs(x_LR_avg) > MIN_DELTA_X_THRESHOLD))
	{
		if ((x_FB_avg < 0) && (x_LR_avg < 0)) return DEG_135_RIGHT;
		if ((x_FB_avg < 0) && (x_LR_avg > 0)) return DEG_135_LEFT;
		if ((x_FB_avg > 0) && (x_LR_avg < 0)) return DEG_45_RIGHT;
		if ((x_FB_avg > 0) && (x_LR_avg > 0)) return DEG_45_LEFT;
	}
	return DONT_MOVE;
}

/*
 * These functions are called within the switch of the function "move_to_sound"
 * and make the robot turn towards the sound source, blink BODY_LED multiple
 * times to indicate the direction and finally spin back to its original
 * orientation
 */

void move_deg_0 (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	go ();
}

void move_deg_45_right (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	eight_times_two_turns (DEG_45_TURN, RIGHT_TURN, MOTOR_SPEED);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	eight_times_two_turns (DEG_45_TURN, LEFT_TURN, MOTOR_SPEED);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	go ();
}

void move_deg_90_right (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (DEG_90_TURN, RIGHT_TURN);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (DEG_90_TURN, LEFT_TURN);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	go ();
}

void move_deg_135_right (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (DEG_90_TURN, RIGHT_TURN);
	eight_times_two_turns (DEG_45_TURN, RIGHT_TURN, MOTOR_SPEED);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	eight_times_two_turns (DEG_45_TURN, LEFT_TURN, MOTOR_SPEED);
	quarter_turns (DEG_90_TURN, LEFT_TURN);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	go ();
}

void move_deg_180 (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (DEG_180_TURN, RIGHT_TURN);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (DEG_180_TURN, RIGHT_TURN);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	go ();
}

void move_deg_45_left (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	eight_times_two_turns (DEG_45_TURN, LEFT_TURN, MOTOR_SPEED);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	eight_times_two_turns (DEG_45_TURN, RIGHT_TURN, MOTOR_SPEED);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	go ();
}

void move_deg_90_left (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (DEG_90_TURN, LEFT_TURN);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (DEG_90_TURN, RIGHT_TURN);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	go ();
}

void move_deg_135_left (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (1, LEFT_TURN);
	eight_times_two_turns (2, LEFT_TURN, MOTOR_SPEED);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (1, RIGHT_TURN);
	eight_times_two_turns (2, RIGHT_TURN, MOTOR_SPEED);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	go ();
}

/*
 * Function that selects one of the 8 previous "move" functions depending
 * on where the sound is coming from
 */

void move_to_sound (void)
{
	//Saves initial positions of the motor
	int32_t right_motor_pos = right_motor_get_pos ();
	int32_t left_motor_pos = left_motor_get_pos ();

	switch (determine_sound_state ())
	{
		case DEG_0 :
			move_deg_0 ();
			break;
		case DEG_45_RIGHT :
			move_deg_45_right ();
			break;
		case DEG_90_RIGHT :
			move_deg_90_right ();
			break;
		case DEG_135_RIGHT :
			move_deg_135_right ();
			break;
		case DEG_180 :
			move_deg_180 ();
			break;
		case DEG_45_LEFT :
			move_deg_45_left ();
			break;
		case DEG_90_LEFT :
			move_deg_90_left ();
			break;
		case DEG_135_LEFT :
			move_deg_135_left ();
			break;
		case DONT_MOVE :
			break;
	}
	right_motor_set_pos (right_motor_pos);
	left_motor_set_pos (left_motor_pos);
}

/*
 * Function that determines from which direction
 * the sound is coming from
 */

void determine_sound_origin (void)
{
	float time_shift_FB = determine_argument (micFront_cmplx_input) -
					      determine_argument (micBack_cmplx_input);

	float time_shift_LR = determine_argument (micLeft_cmplx_input) -
						  determine_argument (micRight_cmplx_input);

	float x_FB = (float)time_shift_FB * SOUND_SPEED * (FFT_SIZE/(20*M_PI*max_norm_index));
	float x_LR = (float)time_shift_LR * SOUND_SPEED * (FFT_SIZE/(20*M_PI*max_norm_index));

	if (counter_x < SAMPLE_SIZE)
	{
		if ((abs(x_FB) < EPUCK_DIAMETER) &&
			(abs(x_LR) < EPUCK_DIAMETER))
		{
			 tab_x_FB[counter_x] = x_FB;
			 tab_x_LR[counter_x] = x_LR;
			 ++counter_x;
		}
	} else {
		x_FB_avg = 0;
		x_LR_avg = 0;
		for (int8_t i = 0; i < SAMPLE_SIZE; ++i)
		{
			x_FB_avg += tab_x_FB[i];
			x_LR_avg += tab_x_LR[i];
		}
		x_FB_avg = (float)(x_FB_avg/SAMPLE_SIZE);
		x_LR_avg = (float)(x_LR_avg/SAMPLE_SIZE);
		move_to_sound ();
		counter_x = 0;
	}
}

/*
 * FUNCTION USED TO TEST TO SEE IF A THREAD CAN BE CALLED WITHIN A THREAD
 * TO BE DELETED AFTERWARDS
 */

void freq350_handler (void)
{
	/*
	if (get_moving ())
	{
		palTogglePad(GPIOB, GPIOB_LED_BODY);
		//suspend_TOF ();
		stop ();
		set_moving (0);
	} else
	{
		palTogglePad(GPIOB, GPIOB_LED_BODY);
		go ();
		//resume_TOF ();
		set_moving (1);
	}
	chThdSleepMilliseconds(1000);
	*/
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	for (int8_t i = 0; i < 6; ++i)
	{
		delay (HALF_SECOND);
	}
	go ();
	palTogglePad(GPIOB, GPIOB_LED_BODY);
}

/*
 * Makes the robot spin once in one direction and then spins one more time in
 * the other direction (called when one of the frequencies is perceived)
 */

void  spin_left_then_right (void)
{
	stop();
	int32_t right_motor_pos = right_motor_get_pos ();
	int32_t left_motor_pos = left_motor_get_pos ();
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (ONE_TURN, LEFT_TURN);
	quarter_turns (ONE_TURN, RIGHT_TURN);
	stop();
	right_motor_set_pos (right_motor_pos);
	left_motor_set_pos (left_motor_pos);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
}

/*
 * Stops the robot for a couple of seconds and makes it move again
 */

void stop_and_go (void)
{
//	//static systime_t start_time;
//	if (moving == GO)
//	{
//		//start_time = //prends temps
//		palTogglePad(GPIOB, GPIOB_LED_BODY);
//		stop ();
//		moving = STOP;
//	} else
//	{
//		//if(/*time*/ - start_time > THRESHHOLD){
//			palTogglePad(GPIOB, GPIOB_LED_BODY);
//			go ();
//			moving = GO;
//		//}
//	}

	//chSysLock();
//	chThdSuspendS(&tofThd);
//	chSysUnlock();
//	chSysLock();
//	chThdSuspendS(&mainThread);
	//chSysUnlock();
	//int32_t right_motor_pos = right_motor_get_pos ();
	//int32_t left_motor_pos = left_motor_get_pos ();
	stop();
	waitThd = chThdCreateStatic(waWaitThd, sizeof(waWaitThd), NORMALPRIO+50, WaitThd, NULL);

	//palTogglePad(GPIOB, GPIOB_LED_BODY);
	//delay(10*HALF_SECOND);
	//palTogglePad(GPIOB, GPIOB_LED_BODY);
	//right_motor_set_pos (right_motor_pos);
	//left_motor_set_pos (left_motor_pos);
	//chSysUnlock();

//	chThdResume(&tofThd, (msg_t)0x1337);

}

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*
*	param : data = buffer containing the magnitudes of a microphone
*/
void sound_remote(float* data)
{
	float max_norm = MIN_VALUE_THRESHOLD;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++)
	{
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//350 OK
	//406 OK, MOST PROBABLY WONT BE USED SO TO BE REMOVED
	//1150 OK
	//1400 OK

	if (max_norm_index >= FREQ_350_L && max_norm_index <= FREQ_350_H)
	{
		chprintf((BaseSequentialStream *)&SD3, "350\r\n\n");
		stop_and_go ();
		//chThdSleepMilliseconds(1000);
	}

	else if (max_norm_index >= FREQ_406_L && max_norm_index <= FREQ_406_H)
	{
		spin_left_then_right ();
	}

	else if (max_norm_index >= FREQ_1150_L && max_norm_index <= FREQ_1150_H)
	{

	}

	else if (max_norm_index >= FREQ_1400_L && max_norm_index <= FREQ_1400_H)
	{
		determine_sound_origin ();
	}
}

/*
 * Function that performs the FFT and determines the magnitude of each
 * DFT for every microphone => makes the "process_audio" function easier
 * to read
 */
void compute_fft_and_mag (void)
{
	doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
	doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
	doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
	doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

	arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
	arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
	arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
	arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
}

/*
 * Function that correctly fills the microphone arrays => makes the "process_audio"
 * function easier to read
 *
 * params : int16_t *data : Buffer containing 4 times 160 samples. the samples are sorted by micro
 *							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
 *			uint16_t num_samples : Tells how many data we get in total (should always be 640)
 */
void fill_arrays (int16_t *data, uint16_t i)
{
	//construct an array of complex numbers. Put 0 to the imaginary part
	micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
	micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
	micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
	micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

	nb_samples++;

	micRight_cmplx_input[nb_samples] = 0;
	micLeft_cmplx_input[nb_samples] = 0;
	micBack_cmplx_input[nb_samples] = 0;
	micFront_cmplx_input[nb_samples] = 0;

	nb_samples++;
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples)
{
	chThdSetPriority(NORMALPRIO+12);
	//int motor_pos = right_motor_get_pos();

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4)
	{
		fill_arrays (data, i);

		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){

		compute_fft_and_mag ();


		if(mustSend > 1){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}

		nb_samples = 0;
		mustSend++;

		sound_remote(micLeft_output);
	}
	//right_motor_set_pos(motor_pos);
}









