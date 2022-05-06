#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


//To be able to retrieve the value of moving in other modules
int8_t get_moving (void);

//To be able to affect the value of moving from other modules
void set_moving (int8_t new_moving);

//Microphone thread funtion
void processAudioData(int16_t *data, uint16_t num_samples);

#endif /* AUDIO_PROCESSING_H */
