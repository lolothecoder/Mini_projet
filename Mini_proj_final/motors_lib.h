#ifndef MOTORS_LIB_H
#define MOTORS_LIB_H
#define LEFT_TURN			1
#define RIGHT_TURN			-1
#define STRAIGHT			1
#define BACK				-1
#define MOTOR_SPEED   		600 // []
#define SINGLE_TURN			1

// Initialize right motor position to 0
//IMPORTANT TO CALL BEFORE THE WHILE LOOP
void init_pos_motor(void);

//Rolls in a straight line for length distance
void straight_line(uint8_t distance, int dir);

//Turns num_of_quarter_turns * 90 degrees
void quarter_turns(uint8_t num_of_quarter_turns, int dir);

//Turns num_of_quarter_turns * 22.5 degrees
void eight_times_two_turns(uint8_t num_of_sixteen_turns, int dir, uint16_t speed);

//Rolls straight for length distance then turn 90 degrees
void straight_then_turn(uint8_t distance);

//Initializes loop thread
void loop_start(void);

//Sets speed to 0
void stop (void);

//Sets the speed to MOTOR_SPEED
void go (void);

void set_speed(int speed);

int dist_to_steps(int distance);

int steps_to_dist(int steps);

void infinite_stop(void);

int conditional_advance(uint8_t distance, uint8_t dir,  bool continue_advance);

#endif /* MOTORS_LIB_H */
