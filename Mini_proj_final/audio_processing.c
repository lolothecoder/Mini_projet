#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors_lib.h>
#include "motors.h"
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

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

#define TWO_TURNS			8
#define GO					1
#define STOP				0
#define MIN_VALUE_THRESHOLD	10000 

#define SAMPLE_SIZE				10
#define MIN_DELTA_X_THRESHOLD	10 //If the delta x value is smaller than this threshold
								   //then the sound is considered to be perpendicular to
								   //the microphone axis in question

//Arrays containing delta x values
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

/*
 * Gives us the current frequency that's being detected
 */

static int16_t max_norm_index = -1;


#define TWO_TURNS			8
#define GO					1
#define STOP				0
#define MIN_VALUE_THRESHOLD	10000 

//we don't analyze before this index to not use resources for nothing
#define MIN_FREQ		10

#define FREQ_RIGHT		23	//359Hz
#define FREQ_900		58  //900Hz
#define FREQ_1150		74	//1150Hz
#define FREQ_1400		90  //1400Hz


//we don't analyze after this index to not use resources for nothing
#define MAX_FREQ		100

//Lower and upper bounds used within the sound_remote function
#define FREQ_RIGHT_L		(FREQ_RIGHT-2)
#define FREQ_RIGHT_H		(FREQ_RIGHT+2)
#define FREQ_1400_L			(FREQ_1400 -2)
#define FREQ_1400_H			(FREQ_1400 +2)
#define FREQ_900_L			(FREQ_900 -2)
#define FREQ_900_H			(FREQ_900 +2)
#define FREQ_1150_L			(FREQ_1150 -2)
#define FREQ_1150_H			(FREQ_1150 +2)

#define EPUCK_DIAMETER		73 //value in mm
#define SOUND_SPEED			343 //value in m/s
#define WHISTLE_DISTANCE	10 //value in cm


/*
 * Simple Delay Function
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
	delay (SystemCoreClock/32);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (SystemCoreClock/32);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (SystemCoreClock/32);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (SystemCoreClock/32);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (SystemCoreClock/32);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (SystemCoreClock/32);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	delay (SystemCoreClock/32);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
}

/*
 * Function that determines the argument of a complex value :
 */

float determin_argument (float* data_dft)
{
	float ratio = (float)(data_dft[2*max_norm_index+1]/data_dft[2*max_norm_index]);
	ratio = ratio - ((float)((ratio*ratio*ratio)/3));

	//Approximation of arctan
	return (ratio - (float)((ratio*ratio*ratio)/3));
}

/*
 * Function that determines the sound orientation state for the switch
 * within the change orientation function
 */

SOUND_ORIENTATION_t determin_sound_state (void)
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
 * These functions are called within the switch and make the robot
 * move towards the sound source
 */

void move_deg_0 (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	straight_line (WHISTLE_DISTANCE, STRAIGHT);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	go ();
}

void move_deg_45_right (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	eight_times_two_turns (2, RIGHT_TURN, MOTOR_SPEED);
	straight_line (WHISTLE_DISTANCE, STRAIGHT);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	go ();
}

void move_deg_90_right (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (1, RIGHT_TURN);
	straight_line (WHISTLE_DISTANCE, STRAIGHT);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	go ();
}

void move_deg_135_right (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (1, RIGHT_TURN);
	eight_times_two_turns (2, RIGHT_TURN, MOTOR_SPEED);
	straight_line (WHISTLE_DISTANCE, STRAIGHT);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	go ();
}

void move_deg_180 (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (2, RIGHT_TURN);
	straight_line (WHISTLE_DISTANCE, STRAIGHT);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	go ();
}

void move_deg_45_left (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	eight_times_two_turns (2, LEFT_TURN, MOTOR_SPEED);
	straight_line (WHISTLE_DISTANCE, STRAIGHT);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	go ();
}

void move_deg_90_left (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (1, LEFT_TURN);
	straight_line (WHISTLE_DISTANCE, STRAIGHT);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	go ();
}

void move_deg_135_left (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (1, LEFT_TURN);
	eight_times_two_turns (2, LEFT_TURN, MOTOR_SPEED);
	straight_line (WHISTLE_DISTANCE, STRAIGHT);
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	stop ();
	blink_body ();
	go ();
}

/* Function that orients the robot to the direction that the
 * sound is coming from + move 10 cm towards it
 * FINAL ONE
 */

void move_to_sound (void)
{
	switch (determin_sound_state ())
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
}

/*
 * Function that determines from which direction
 * the sound is coming from
 */

void determin_sound_origin (void)
{
	float time_shift_FB = determin_argument (micFront_cmplx_input) -
					      determin_argument (micBack_cmplx_input);

	float time_shift_LR = determin_argument (micLeft_cmplx_input) -
						  determin_argument (micRight_cmplx_input);

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
 * Function that handles what the robot should do when a sound around 900Hz
 * is perceived :
 * Stops the robot if it's moving and starts it if it's not moving
 */

void freq900_handler (void)
{
	determin_sound_origin ();
}

/*
 * Function that handles what the robot should do when a sound around 1150Hz
 * is perceived :
 * Makes the robot spin for 3 turns
 */

void  freq1150_handler (void)
{
	palTogglePad(GPIOB, GPIOB_LED_BODY);
	quarter_turns (TWO_TURNS, LEFT_TURN);
	quarter_turns (TWO_TURNS, RIGHT_TURN);
	go ();
	palTogglePad(GPIOB, GPIOB_LED_BODY);
}

/*
 * Function that handles what the robot should do when a sound around 1400Hz
 * is perceived :
 * Stop/go
 */

void freq1400_handler (void)
{
	if (get_moving ())
	{
		palTogglePad(GPIOB, GPIOB_LED_BODY);
		stop ();
		set_moving (0);
	} else
	{
		palTogglePad(GPIOB, GPIOB_LED_BODY);
		go ();
		set_moving (1);
	}
	chThdSleepMilliseconds(1000);
}

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
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

	if (max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H)
	{
		//determin_sound_origin ();
	}

	if (max_norm_index >= FREQ_900_L && max_norm_index <= FREQ_900_H)
	{
		//freq900_handler ();
	}

	if (max_norm_index >= FREQ_1150_L && max_norm_index <= FREQ_1150_H)
	{
		//freq1150_handler ();
	}

	if (max_norm_index >= FREQ_1400_L && max_norm_index <= FREQ_1400_H)
	{
		freq1400_handler ();
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
	chThdSetPriority(NORMALPRIO+2);
	//int motor_pos = right_motor_get_pos();
	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4)
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

		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){

		compute_fft_and_mag ();

		if(mustSend > 8){
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

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

