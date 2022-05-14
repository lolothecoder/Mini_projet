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

//Microphone thread funtion
void processAudioData(int16_t *data, uint16_t num_samples);

uint8_t get_loop_distance (void);
void delay(unsigned int n);

#endif /* AUDIO_PROCESSING_H */
