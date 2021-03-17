/*
 * RSLK_Rover.c
 *
 *  Created on: Mar 5, 2021
 *      Author: wills
 */
#include "RSLK_Rover.h"

//Handler for encoder interrupts
void PORT5_IRQHandler(void) //ERB
{
    //left or right wheel interrupt, since they are both on the same port
    if(encBInterruptCheck(&right_wheel_data))//right wheel
    {
        //Calculate time since last interrupt
        if(right_wheel_data.wheel_state != STOPPED)
            right_wheel_data.enc_period = right_wheel_data.t32_period_count;
        else
            right_wheel_data.enc_period = right_wheel_data.sys_clk;

        //Reset Timer32_1_base
        right_wheel_data.t32_period_count = 0;

        //Reset stop timer
        right_wheel_data.t32_stop_count = 0;

        //check if ERA is high
        if(GPIO_getInputPinValue(GPIO_PORT_P10, GPIO_PIN4))
        {
            right_wheel_data.wheel_state = BACKWARD;
        }
        else
        {
            right_wheel_data.wheel_state = FORWARD;
        }

    }
    else if(encBInterruptCheck(&left_wheel_data)) //check if left wheel interrupt
    {
        //Calculate time since last interrupt
        if(left_wheel_data.wheel_state != STOPPED)
            left_wheel_data.enc_period = left_wheel_data.t32_period_count;
        else
            left_wheel_data.enc_period = left_wheel_data.sys_clk;

        //Reset Timer32_1_base
        left_wheel_data.t32_period_count = 0;

        //Reset stop timer
        left_wheel_data.t32_stop_count = 0;

        //check if ERA is high
        if(GPIO_getInputPinValue(GPIO_PORT_P10, GPIO_PIN5))
        {
            left_wheel_data.wheel_state = BACKWARD;
        }
        else
        {
            left_wheel_data.wheel_state = FORWARD;
        }
    }

    clearEncBInterrupt();
}

void T32_INT2_IRQHandler(void)
{

    right_wheel_data.t32_period_count++;
    right_wheel_data.t32_stop_count++;

    left_wheel_data.t32_period_count++;
    left_wheel_data.t32_stop_count++;

    if(rover_state != ROVER_STOPPED && move_end_cond == TIME)
        move_timer.count++;

    Timer32_clearInterruptFlag(RSLK_TIMER32_BASE);
}

void stopRover()
{
    setWheelDutyCycle(&right_wheel_data, 0);
    setWheelDutyCycle(&left_wheel_data, 0);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = ROVER_STOPPED;
    move_end_cond = NONE;
}

void checkEndCond()
{
    if(rover_state != ROVER_STOPPED)
    {
        switch(move_end_cond)
        {
        case TIME:      if((move_timer.count / T32_TICK_TO_MS) > move_timer.target_count)
                        {
                            move_timer.count = 0;
                            stopRover();
                        }
                        break;
        case ROTATION: break;
        default: break;
        }
    }
}

void clearEncBInterrupt()
{
    GPIO_clearInterruptFlag(right_wheel_data.wheel_encb_pin.portNum, right_wheel_data.wheel_encb_pin.pinNum);
    GPIO_clearInterruptFlag(left_wheel_data.wheel_encb_pin.portNum, left_wheel_data.wheel_encb_pin.pinNum);
}

void initRSLKRover(uint32_t _sys_clk)
{
    rover_state = ROVER_STOPPED;
    move_end_cond = NONE;
    move_timer.count = 0;
    move_timer.target_count = 0;
    initRightRSLKWheel(&right_wheel_data, _sys_clk, T32_1_COUNT);
    initLeftRSLKWheel(&left_wheel_data, _sys_clk, T32_1_COUNT);
    Interrupt_enableMaster();
}

void initRSLKTimer32(uint32_t rslk_timer32_base)
{
    uint32_t timer32_irq_handle;
    switch(rslk_timer32_base)
    {
    case TIMER32_1_BASE: timer32_irq_handle = INT_T32_INT2; break;
    case TIMER32_0_BASE: timer32_irq_handle = INT_T32_INT1; break;
    default: while(1){} //incorrect timer32 base value
    }

    Timer32_initModule(rslk_timer32_base, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_setCount(rslk_timer32_base, T32_1_COUNT);
    Timer32_enableInterrupt(rslk_timer32_base);
    Interrupt_enableInterrupt(timer32_irq_handle);
    Timer32_startTimer(rslk_timer32_base, CONTINUOUS);
}

void moveForwardForTime(uint32_t _rpm, uint32_t milliSecs)
{
    //nominal _rpm unit is deci rpm
    move_timer.target_count = milliSecs;
    move_end_cond = TIME;
    double dutyCycle = ((double)(_rpm) + RPM_OFFSET * UNIT_FACTOR) / RPM_PER_DC / UNIT_FACTOR;
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirFwrd(&right_wheel_data);
    setWheelDirFwrd(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = MOVING_FWD;
}
