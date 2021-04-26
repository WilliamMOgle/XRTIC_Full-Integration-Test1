/*
 * RSLK_Rover.h
 *
 *  Created on: Mar 5, 2021
 *      Author: wills
 */

#ifndef RSLK_ROVER_H_
#define RSLK_ROVER_H_

#include "RSLK_Wheel.h"
#include "uart_HAL.h"

//DEFINES///////////////////////////////
//Timer 32 definitions
#define RSLK_TIMER32_BASE TIMER32_1_BASE
#define T32_1_PERIOD        0.00001
#define COMP_SAMPLING_PERIOD    10000 //COMP_SAMPLING_PERIOD * T32_1_PERIOD = actual sampling period in seconds
#define RIGHT_WHEEL_COMP_GAIN 80    //take inverse
#define LEFT_WHEEL_COMP_GAIN  80    //take inverse
#define COMP_SCALAR            100    //scalar so not as many floating point variables need to be used
#define DEF_PRESCALER       TIMER32_PRESCALER_1
#define DEF_COUNT_SIZE      TIMER32_32BIT
#define DEF_MODE            TIMER32_PERIODIC_MODE
#define COMP_OVERSHOOT_MAX  20000
#define INIT_ELAPSED_CYCLES_COUNT 0
#define INIT_WAIT_CYCLES_COUNT 0
#define CONTINUOUS false
#define ONE_SHOT true

//Constants used for non-compensated rover move functions
//RPM = 276.2*DC - 4.187
#define RPM_OFFSET      4.187    //offset in DC to RPM conversion equation
#define RPM_PER_DC      276.2

#define T32_TICK_TO_MS  100//0.001 / T32_1_PERIOD

#define ROVER_WIDTH_TO_WHEEL_DIAM_RATIO     2
#define WHEEL_RADIUS       35           //MILLIMETERS
#define DEG_PER_RAD   57

#define MAX_WHEEL_RPM 260
#define MIN_WHEEL_RPM 0
#define MAX_DUTY_CYCLE 1
#define MIN_DUTY_CYCLE 0.02

#define REAL_TIME_ROVER_STATE_TRACKING false    //enable to this to update the Rover_State struct based on the real-time wheel states


//DATA TYPE DEFINTIONS//////////////////////////
typedef enum {
    MOVING_FORWARD,
    MOVING_BACKWARD,
    ROTATING_RIGHT,
    ROTATING_LEFT,
    ROVER_STOPPED,
    ABNORMAL
}Rover_State;       //tracks what movement state the rover is in

typedef enum {
    NONE,
    TIME,
    ROTATION,
    DISTANCE
}Move_End_Cond;     //tracks what the end condition of the move is

typedef struct {
    uint32_t count;
    uint32_t target_count;
}Move_Timer;

typedef struct {
    uint32_t current_rotation;      //Integer degrees, always positive
    uint32_t desired_degree;
}Rover_Rotation_Tracker;

typedef struct {
    uint32_t sys_clk_freq;      //frequency of system clock in Hz
    double timer_period;        //desired timer period in seconds
    uint32_t count_max;         //value t32 counts down from to achieve 'timer_period'
}T32_Data;


//GLOBAL VARIABLE DECLARATIONS////////////////////////////
RSLK_Wheel right_wheel_data;
RSLK_Wheel left_wheel_data;
Move_Timer move_timer;
Rover_Rotation_Tracker rover_rotation_tracker;
Rover_State rover_state;
Move_End_Cond move_end_cond;
T32_Data t32_data;


//FUNCTION PROTOTYPES///////////////////////////////
//Rover Init
void initRSLKRover(uint32_t _sys_clk);
void initRSLKTimer32();
//Rover Move Commands
//time
void moveForwardForTime(uint32_t, uint32_t);      //input a int in units of deci rpm
void moveBackwardForTime(uint32_t, uint32_t);
void rotateRightForTime(uint32_t, uint32_t);
void rotateLeftForTime(uint32_t, uint32_t);
void moveForwardForTimeComp(uint32_t, uint32_t);      //input a int in units of deci rpm
void moveBackwardForTimeComp(uint32_t, uint32_t);
void rotateRightForTimeComp(uint32_t, uint32_t);
void rotateLeftForTimeComp(uint32_t, uint32_t);
//indefinitely
void moveForwardIndefinitely(uint32_t);
void moveBackwardIndefinitely(uint32_t);
void rotateRightIndefinitely();
void rotateLeftIndefinitely();
void moveForwardIndefinitelyComp(uint32_t);
void moveBackwardIndefinitelyComp(uint32_t);
void rotateRightIndefinitelyComp();
void rotateLeftIndefinitelyComp();
//physical parameter
void moveForwardForDistance(uint32_t, uint32_t);    //millimeters
void moveBackwardForDistance(uint32_t, uint32_t);   //millimeters
void rotateRightForDegrees(uint32_t,uint32_t);      //overall rover degrees
void rotateLeftForDegrees(uint32_t,uint32_t);       //overall rover degrees
void moveForwardForDistanceComp(uint32_t, uint32_t);    //millimeters
void moveBackwardForDistanceComp(uint32_t, uint32_t);   //millimeters
void rotateRightForDegreesComp(uint32_t,uint32_t);      //overall rover degrees
void rotateLeftForDegreesComp(uint32_t,uint32_t);       //overall rover degrees
//Enable
void disableRover();
void enableRover();
void enableCompensator();
void disableCompensator();
//Stop
void stopRover();
void hardStop();
void hardStopRight();
void hardStopLeft();
//Non-blocking updates
void checkEndCond();
void updateRoverState();
void compensator();
//Helper functions
bool rotationConditionMet();
bool timeConditionMet();
void resetRotationTracker();
void resetTimeTracker();
uint32_t rotationDegConversion(uint32_t);
void currentRoverRotation();
double rawConvertRPMToDutyCycle(uint32_t _rpm);
void clearEncBInterrupt();


#endif /* RSLK_ROVER_H_ */
