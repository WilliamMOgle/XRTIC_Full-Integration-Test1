/*
 * RSLK_Rover.h
 *
 *  Created on: Mar 5, 2021
 *      Author: wills
 */

#ifndef RSLK_ROVER_H_
#define RSLK_ROVER_H_

#include "RSLK_Wheel.h"

//Timer 32 definitions
#define RSLK_TIMER32_BASE TIMER32_1_BASE
#define T32_1_COUNT         480
//#define TIMER32_INIT_COUNT 4800 //0.1ms for 48MHz clock
#define INIT_ELAPSED_CYCLES_COUNT 0
#define INIT_WAIT_CYCLES_COUNT 0
#define CONTINUOUS false
#define ONE_SHOT true

//RPM = 276.2*DC - 4.187
#define RPM_OFFSET      4.187    //offset in DC to RPM conversion equation
#define RPM_PER_DC      276.2

#define T32_TICK_TO_MS  100

typedef enum {
    MOVING_FWD,
    MOVING_BKWD,
    ROTATING_CCW,
    ROTATING_CW,
    ROVER_STOPPED
}Rover_State;       //tracks what movement state the rover is in

typedef enum {
    NONE,
    TIME,
    ROTATION
}Move_End_Cond;     //tracks what the end condition of the move is

typedef struct {
    uint32_t count;
    uint32_t target_count;
}Move_Timer;

RSLK_Wheel right_wheel_data;
RSLK_Wheel left_wheel_data;
Move_Timer move_timer;
Rover_State rover_state;
Move_End_Cond move_end_cond;

void initRSLKRover(uint32_t _sys_clk);
//void initRightInt();
//void initLeftInt();
void initRSLKTimer32(uint32_t);
void clearEncBInterrupt();
void moveForwardForTime(uint32_t, uint32_t);      //input a int in units of deci rpm
void checkEndCond();
void stopRover();




#endif /* RSLK_ROVER_H_ */
