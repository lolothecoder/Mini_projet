#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors_lib.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

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

#define MIC_COUNT		4	//number of microphones

//we don't analyze before this index to not use resources for nothing
#define MIN_FREQ		10

#define FREQ_RIGHT		23	//359Hz
#define FREQ_900		58  //900Hz
#define FREQ_1150		74	//1150Hz
#define FREQ_1400		90  //1400Hz
//#define FREQ_1800		115 //1800Hz

//we don't analyze after this index to not use resources for nothing
#define MAX_FREQ		130

//Lower and upper bounds used within the sound_remote function
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_1400_L			(FREQ_1400 -2)
#define FREQ_1400_H			(FREQ_1400 +2)
#define FREQ_900_L			(FREQ_900 -2)
#define FREQ_900_H			(FREQ_900 +2)
#define FREQ_1150_L			(FREQ_1150 -2)
#define FREQ_1150_H			(FREQ_1150 +2)
//#define FREQ_1800_L			(FREQ_1800 -4)
//#define FREQ_1800_H			(FREQ_1800 +4)

#define EPUCK_DIAMETER		7.3 //value in cm
#define SOUND_SPEED			343 //value in m/s


/*
 * Function that determines the argument of a complex value
 */

float determin_argument (float* data_mag, float* data_dft)
{
	//float max_norm = MIN_VALUE_THRESHOLD;
	float ratio =0;

	//int16_t max_norm_index = -1;

	//search for the highest peak
	/*
	for (uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++)
	{
		if (data_mag[i] > max_norm)
		{
			max_norm = data_mag[i];
			max_norm_index = i;
		}
	}

	ratio = (float)(data_dft[2*max_norm_index+1]/data_dft[2*max_norm_index]);
	*/

	for (uint16_t i = 0; i < FFT_SIZE/2; ++i)
	{
		ratio += (float)(data_dft[2*i+1]/data_dft[2*i]);
	}

	ratio = ratio/FFT_SIZE;


	//Approximation of arctan
	return (ratio - (float)((ratio*ratio*ratio)/3));
}

/*
 * Function that determines from which direction
 * the sound is coming from
 */

void determin_sound_origin (void)
{
	float time_shift_FB = determin_argument (micFront_output, micFront_cmplx_input) -
					      determin_argument (micBack_output, micBack_cmplx_input);

	time_shift_FB = (float)(time_shift_FB/((2*M_PI)/FFT_SIZE));

	float time_shift_LR = determin_argument (micLeft_output, micLeft_cmplx_input) -
						  determin_argument (micRight_output, micRight_cmplx_input);

	time_shift_LR = (float)(time_shift_LR/((2*M_PI)/FFT_SIZE));

	float cos_omega_FB = (float)((SOUND_SPEED*time_shift_FB*100)/EPUCK_DIAMETER);
	float cos_omega_LR = (float)((SOUND_SPEED*time_shift_LR*100)/EPUCK_DIAMETER);

	chprintf((BaseSequentialStream *)&SD3, "time shift FB = %d%\r\n\n", time_shift_FB);
	chprintf((BaseSequentialStream *)&SD3, "time shift LR = %d%\r\n\n", time_shift_LR);
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
 * Function that handles what the robot should do when a sound around 1800Hz
 * is perceived :
 * determines sound origin and goes towards it
 * NOT IMPLEMENTED YET
 */
/*
void freq1800_handler (void)
{
	chprintf((BaseSequentialStream *)&SD3, "In freq1800 %d%\r\n\n");
	if (get_moving ())
	{
		palTogglePad(GPIOB, GPIOB_LED_BODY);
		stop ();
		set_moving (STOP);
	} else
	{
		palTogglePad(GPIOB, GPIOB_LED_BODY);
		go ();
		set_moving (GO);
	}
	chThdSleepMilliseconds(1000);
}
*/

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

void sound_remote(float* data)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

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
		determin_sound_origin ();
	}

	if (max_norm_index >= FREQ_900_L && max_norm_index <= FREQ_900_H)
	{
		freq900_handler ();
	}

	if (max_norm_index >= FREQ_1150_L && max_norm_index <= FREQ_1150_H)
	{
		freq1150_handler ();
	}

	if (max_norm_index >= FREQ_1400_L && max_norm_index <= FREQ_1400_H)
	{
		freq1400_handler ();
	}
	/*
	if (max_norm_index >= FREQ_1800_L && max_norm_index <= FREQ_1800_H)
	{
		freq1800_handler ();
	}
	*/
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
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}

static THD_WORKING_AREA(waThdFrontLed, 128);
static THD_FUNCTION(ThdFrontLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();
        palTogglePad(GPIOD, GPIOD_LED_FRONT);
        chThdSleepUntilWindowed(time, time + MS2ST(1000));
    }
}

void front_led_start (void)
{
	 chThdCreateStatic(waThdFrontLed, sizeof(waThdFrontLed), NORMALPRIO, ThdFrontLed, NULL);
}



