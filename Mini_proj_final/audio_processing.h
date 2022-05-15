/*
 * ATTENTION
 * The files audio_processing.c and audio_processing.h had been originally taken from "TP5_Noisy"
 * and were used as a base for this module of the project. Some of the #define, static variables
 * and the functions process_audio_data and sound_remote were take from that TP but have been modified
 * and other elements have been removed due to them not being necessary for our project (get_buffer (),
 * wait_send_to_computer (), typdef of different microphones...). Other elements were added to be able
 * to complete our project.
 */

#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#ifdef __cplusplus
extern "C" {
#endif

#define HALF_SECOND 		SystemCoreClock/8

/*
 * Enum used to identify which top LEDs should be turned on/off to indicate
 * what is the current loop distance ; used within the functions
 * "decrease_loop_distance" and "select_top_led_configuration"
 *
 * LED CONFIGURATION SIGNIFICANCE :
 * 	- if none of the top LEDs are on => loop_distance = 10 cm ;
 * 	- if one of the top LEDs is on => loop_distance = 20 cm ;
 * 	- if two of the top LEDs are on => loop_distance = 30 cm ;
 * 	- if three of the top LEDs are on => loop_distance = 40 cm.
 */
typedef enum {
	LOOP_10 = 0,
	LOOP_20,
	LOOP_30,
	LOOP_40
} TOP_LED_CONFIGURATION_t;

//Microphone thread funtion
void processAudioData(int16_t *data, uint16_t num_samples);

uint8_t get_loop_distance (void);
void delay(unsigned int n);
void select_top_led_configuration (TOP_LED_CONFIGURATION_t config);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_PROCESSING_H */
