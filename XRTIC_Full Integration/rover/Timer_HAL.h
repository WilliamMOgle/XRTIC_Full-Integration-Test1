/*
 * Timer_HAL.h
 *
 *  Created on: Apr 17, 2019
 *      Author: Willi
 */

#ifndef TIMER_HAL_H_
#define TIMER_HAL_H_

#include <driverlib.h>

#define TIMER32_INIT_COUNT 4800 //0.1ms for 48MHz clock
#define INIT_ELAPSED_CYCLES_COUNT 0
#define INIT_WAIT_CYCLES_COUNT 0
#define CONTINUOUS false
#define ONE_SHOT true

//declare a struct for the SW timer
typedef struct{
    uint32_t hwtimer_p;
    uint32_t waitCycles;    //number of cycles to count before restarting
    uint32_t elapsedCycles; //number of cycles that have been already counted since latest rollover
}OneShot_SW_Timer;

OneShot_SW_Timer SW_Timer_1; //initialize software timer for Music
OneShot_SW_Timer SW_Timer_2; //initialize software timer for other purposes

//FUNCTION PROTOTYPES
//bool Timer0_Expired_ForMusic();
void Init_Timer32_0(unsigned count, bool oneShot);
void initSWTimer1();
void initSWTimer2();
bool Timer32_0_Expired();
bool Timer32_1_Expired();
void resetSWTimer2();
void resetSWTimer1();
void updateTimerA0(uint_fast16_t actionValue, uint32_t CCR);
void updateSW1WaitCycles(uint32_t noteLength);
bool SW1TimerRollover();
void checkTimer32Rollover();
bool SW2TimerRollover();
void updateSW2WaitCycles(unsigned iterations);



#endif /* TIMER_HAL_H_ */
