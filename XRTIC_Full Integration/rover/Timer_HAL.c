/*
 * Timer_HAL.c
 *
 *  Created on: Apr 17, 2019
 *      Author: Willi
 */

#include "Timer_HAL.h"

void initSWTimer1()
{
    SW_Timer_1.hwtimer_p = TIMER32_0_BASE;
    SW_Timer_1.waitCycles = INIT_WAIT_CYCLES_COUNT;
    SW_Timer_1.elapsedCycles = INIT_ELAPSED_CYCLES_COUNT;
}

void initSWTimer2()
{
    SW_Timer_2.hwtimer_p = TIMER32_0_BASE;
    SW_Timer_2.waitCycles = INIT_WAIT_CYCLES_COUNT;
    SW_Timer_2.elapsedCycles = INIT_ELAPSED_CYCLES_COUNT;
}

void resetSWTimer2()
{
    SW_Timer_2.elapsedCycles = 0;
}

void T32_INT1_IRQHandler(void)
{
    SW_Timer_1.elapsedCycles++;
    SW_Timer_2.elapsedCycles++;
    Timer32_clearInterruptFlag(TIMER32_0_BASE);
}

void resetSWTimer1()
{
    SW_Timer_1.elapsedCycles = 0;
}

void Init_Timer32_0(unsigned count, bool oneShot)
{
    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_setCount(TIMER32_0_BASE, count);
    Timer32_enableInterrupt(TIMER32_0_BASE);
    Interrupt_enableInterrupt(INT_T32_INT1);
    Timer32_startTimer(TIMER32_0_BASE, oneShot);

}

bool Timer32_0_Expired()
{
    //initializes the variables
    static unsigned int previousSnap = 0; //static variable remembers value
    unsigned int currentSnap;
    bool returnValue;


    currentSnap = Timer32_getValue(TIMER32_0_BASE); //reads the value of the timer
    returnValue = (currentSnap > previousSnap); //Since the timer counts down, if the current reading is greater than the
                                                //previous reading, it means the timer restarted, which means it expired.
    previousSnap = currentSnap;                 //stores the current reading for the next check
    return returnValue;
}

bool Timer32_1_Expired()
{
    //initializes the variables
    static unsigned int previousSnap = 0; //static variable remembers value
    unsigned int currentSnap;
    bool returnValue;

    currentSnap = Timer32_getValue(TIMER32_1_BASE); //reads the value of the timer
    returnValue = (currentSnap > previousSnap); //Since the timer counts down, if the current reading is greater than the
                                                //previous reading, it means the timer restarted, which means it expired.
    previousSnap = currentSnap;                 //stores the current reading for the next check
    return returnValue;
}

//HAL function for setting the compare value
void updateTimerA0(uint_fast16_t actionValue, uint32_t CCR)
{
    Timer_A_setCompareValue(TIMER_A0_BASE, CCR, actionValue);
}

void updateSW1WaitCycles(uint32_t noteLength)
{
    SW_Timer_1.waitCycles = noteLength;
}

void updateSW2WaitCycles(unsigned iterations)
{
    SW_Timer_2.waitCycles = iterations;
}

bool SW1TimerRollover()
{


    //if the number of times Timer32 has expired (elapsed cycles) is
    //greater than or equal to the designated number of cycles to wait,
    //then the SW timer has rolled over
    //checkTimer32Rollover();
    if(SW_Timer_1.elapsedCycles >= SW_Timer_1.waitCycles)
    {
        SW_Timer_1.elapsedCycles = 0;
        return true;
    }
    //If not, then it has not rolled over
    else
        return false;

}

bool SW2TimerRollover()
{


    //if the number of times Timer32 has expired (elapsed cycles) is
    //greater than or equal to the designated number of cycles to wait,
    //then the SW timer has rolled over
    //checkTimer32Rollover();
    if(SW_Timer_2.elapsedCycles >= SW_Timer_2.waitCycles)
    {
        SW_Timer_2.elapsedCycles = 0;
        return true;
    }
    //If not, then it has not rolled over
    else
        return false;

}

void checkTimer32Rollover()
{
    //checks if Timer32 expired, if so increment elapsed cycles
    if(Timer32_0_Expired())
    {
        SW_Timer_1.elapsedCycles++;
        SW_Timer_2.elapsedCycles++;

    }
}
