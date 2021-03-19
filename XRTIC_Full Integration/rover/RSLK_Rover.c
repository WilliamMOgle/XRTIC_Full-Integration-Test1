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

        //Rotation tracking
        if(rover_state != ROVER_STOPPED && (move_end_cond == ROTATION || move_end_cond == DISTANCE))
            right_wheel_data.wheel_rotation_tracker.ticks++;
        /*UART_transmitData(EUSCI_A0_BASE, (uint8_t)(right_wheel_data.rotation_tracker.ticks/100 + 48));
        UART_transmitData(EUSCI_A0_BASE, (uint8_t)(right_wheel_data.rotation_tracker.ticks/10 - (right_wheel_data.rotation_tracker.ticks/100)*10  + 48));
        UART_transmitData(EUSCI_A0_BASE, (uint8_t)(right_wheel_data.rotation_tracker.ticks - (right_wheel_data.rotation_tracker.ticks/10)*10  + 48));
        UART_transmitData(EUSCI_A0_BASE, '\n');*/
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

        //Rotation tracking
        if(rover_state != ROVER_STOPPED && (move_end_cond == ROTATION || move_end_cond == DISTANCE))
            left_wheel_data.wheel_rotation_tracker.ticks++;

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


void disableRover()
{
    disableWheel(&right_wheel_data);
    disableWheel(&left_wheel_data);
}

void enableRover()
{
    enableWheel(&right_wheel_data);
    enableWheel(&left_wheel_data);
}

void checkEndCond()
{
    if(rover_state != ROVER_STOPPED)
    {
        switch(move_end_cond)
        {
        case TIME:      if(timeConditionMet())
                        {
                            resetTimeTracker();
                            stopRover();
                        }
                        break;
        case ROTATION:  if(rotationConditionMet())
                        {
                            resetRotationTracker();
                            stopRover();
                        }
                        break;
        default: break;
        }
    }
}

bool timeConditionMet()
{
    bool time_cond = false;
    if((move_timer.count / T32_TICK_TO_MS) > move_timer.target_count)
        time_cond = true;

    return time_cond;
}

void resetTimeTracker()
{
    move_timer.count = 0;
    move_timer.target_count = 0;
}

void resetRotationTracker()
{
    rover_rotation_tracker.current_rotation = 0;
    rover_rotation_tracker.desired_degree = 0;
    right_wheel_data.wheel_rotation_tracker.ticks = 0;
    left_wheel_data.wheel_rotation_tracker.ticks = 0;
}

bool rotationConditionMet()
{
    bool rotCond = false;
    currentRoverRotation();
    if(rover_rotation_tracker.current_rotation >= rover_rotation_tracker.desired_degree)
    {
        rotCond = true;
    }
    return rotCond;

}

void currentRoverRotation()
{
    //averaging the rotation counts on both wheels because they should be the same
    rover_rotation_tracker.current_rotation = (right_wheel_data.wheel_rotation_tracker.ticks +
            left_wheel_data.wheel_rotation_tracker.ticks) / 2;
}

void updateRoverState()
{
    updateWheelState(&right_wheel_data);
    updateWheelState(&left_wheel_data);
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
    t32_data.sys_clk_freq = _sys_clk;
    t32_data.timer_period = T32_1_PERIOD;
    t32_data.count_max = _sys_clk * T32_1_PERIOD;
    rover_rotation_tracker.desired_degree = 0;
    rover_rotation_tracker.current_rotation = 0;


    initRightRSLKWheel(&right_wheel_data, t32_data.sys_clk_freq, t32_data.count_max);
    initLeftRSLKWheel(&left_wheel_data, t32_data.sys_clk_freq, t32_data.count_max);
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
    Timer32_setCount(rslk_timer32_base, t32_data.count_max);
    Timer32_enableInterrupt(rslk_timer32_base);
    Interrupt_enableInterrupt(timer32_irq_handle);
    Timer32_startTimer(rslk_timer32_base, CONTINUOUS);
}

double rawConvertRPMToDutyCycle(uint32_t _rpm)
{
    //Nominal rpm unit is deci rpm
    double dutyCycle;
    if(_rpm != 0)
        dutyCycle = ((double)(_rpm) + RPM_OFFSET * UNIT_FACTOR) / RPM_PER_DC / UNIT_FACTOR;
    else
        dutyCycle = 0;

    //If statements constrain rpm conversion function
    if(dutyCycle > MAX_DUTY_CYCLE)
        dutyCycle = MAX_DUTY_CYCLE;
    else if(dutyCycle < MIN_DUTY_CYCLE)
        dutyCycle = 0;

    return dutyCycle;
}

uint32_t rotationDegConversion(uint32_t _deg)
{
    uint32_t conv_deg;
    conv_deg = _deg * ROVER_WIDTH_TO_WHEEL_DIAM_RATIO;
    return conv_deg;
}

uint32_t distanceDegConversion(uint32_t _mm)
{
    uint32_t conv_deg;
    conv_deg = DEG_PER_RAD * _mm / WHEEL_RADIUS;
    return conv_deg;
}

//Time
void moveForwardForTime(uint32_t _rpm, uint32_t milliSecs)
{
    //nominal _rpm unit is deci rpm
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&right_wheel_data);
    setWheelDirForward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = MOVING_FORWARD;
}

void moveBackwardForTime(uint32_t _rpm, uint32_t milliSecs)
{
    //nominal _rpm unit is deci rpm
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirBackward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = MOVING_BACKWARD;
}

void rotateRightForTime(uint32_t _rpm, uint32_t milliSecs)
{
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&left_wheel_data);
    setWheelDirBackward(&right_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = ROTATING_RIGHT;
}

void rotateLeftForTime(uint32_t _rpm, uint32_t milliSecs)
{
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = ROTATING_LEFT;
}

//Indefinitely
void moveForwardIndefinitely(uint32_t _rpm)
{
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&right_wheel_data);
    setWheelDirForward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = MOVING_FORWARD;
    move_end_cond = NONE;
}

void moveBackwardIndefinitely(uint32_t _rpm)
{
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirBackward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = MOVING_BACKWARD;
    move_end_cond = NONE;
}

void rotateRightIndefinitely(uint32_t _rpm)
{
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&left_wheel_data);
    setWheelDirBackward(&right_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = ROTATING_RIGHT;
    move_end_cond = NONE;
}

void rotateLeftIndefinitely(uint32_t _rpm)
{
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = ROTATING_LEFT;
    move_end_cond = NONE;
}

//Physical parameters
void rotateRightForDegrees(uint32_t _rpm, uint32_t degrees)
{
    rover_rotation_tracker.desired_degree = rotationDegConversion(degrees);
    rover_rotation_tracker.current_rotation = 0;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&left_wheel_data);
    setWheelDirBackward(&right_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = ROTATING_RIGHT;
    move_end_cond = ROTATION;
}

void rotateLeftForDegrees(uint32_t _rpm, uint32_t degrees)
{
    rover_rotation_tracker.desired_degree = rotationDegConversion(degrees);
    rover_rotation_tracker.current_rotation = 0;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = ROTATING_LEFT;
    move_end_cond = ROTATION;
}

void moveForwardForDistance(uint32_t _rpm, uint32_t mm)
{
    rover_rotation_tracker.desired_degree = distanceDegConversion(mm);
    rover_rotation_tracker.current_rotation = 0;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&right_wheel_data);
    setWheelDirForward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = MOVING_FORWARD;
    move_end_cond = ROTATION;
}

void moveBackwardForDistance(uint32_t _rpm, uint32_t mm)
{
    rover_rotation_tracker.desired_degree = distanceDegConversion(mm);
    rover_rotation_tracker.current_rotation = 0;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirBackward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = MOVING_BACKWARD;
    move_end_cond = ROTATION;
}
