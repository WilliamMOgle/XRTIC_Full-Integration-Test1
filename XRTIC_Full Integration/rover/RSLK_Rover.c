/*
 * RSLK_Rover.c
 *
 *  Created on: Mar 5, 2021
 *      Author: wills
 */
#include "RSLK_Rover.h"

//  PORT_5_IRQHandler
//  Handler for encoder interrupts
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
//  T32_INT2_IRQHandler
//  This Timer32 Interrupt Handler contains all the
//  different software timers that the RSLK needs.
void T32_INT2_IRQHandler(void)
{

    right_wheel_data.t32_period_count++;
    right_wheel_data.t32_stop_count++;

    left_wheel_data.t32_period_count++;
    left_wheel_data.t32_stop_count++;

    if(rover_state != ROVER_STOPPED && move_end_cond == TIME)
        move_timer.count++;

    if(right_wheel_data.enable_compensator)
        right_wheel_data.compensator_count++;

    if(left_wheel_data.enable_compensator)
        left_wheel_data.compensator_count++;

    Timer32_clearInterruptFlag(RSLK_TIMER32_BASE);
}

//FOUR STOP FUNCTIONS//////////////////////////////////

//  stopRover
//  Stops the rover using the compensator
//  inputs:     none
//  outputs:    none
void stopRover()
{
    rover_state = ROVER_STOPPED;
    move_end_cond = NONE;
    right_wheel_data.des_rpm = 0;
    left_wheel_data.des_rpm = 0;
    transmitString("stopRover\n\r");
}

//  hardStop
//  stops the rover by setting the wheel PWM to 0 immediately
//  and without the compensator
//  inputs:     none
//  outputs:    none
void hardStop()
{
    setWheelDutyCycle(&right_wheel_data, 0);
    setWheelDutyCycle(&left_wheel_data, 0);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = ROVER_STOPPED;
    move_end_cond = NONE;
    right_wheel_data.des_rpm = 0;
    left_wheel_data.des_rpm = 0;
    clearStateVariables(&left_wheel_data);
    clearStateVariables(&right_wheel_data);
    transmitString("hardStop\n\r");
}

//  hardStopRight
//  Stops the right wheel without the compensator
//  inputs:     none
//  outputs:    none
void hardStopRight()
{
    setWheelDutyCycle(&right_wheel_data, 0);
    wheelUpdateMove(&right_wheel_data);
    rover_state = ABNORMAL;
    move_end_cond = NONE;
    right_wheel_data.des_rpm = 0;
    clearStateVariables(&right_wheel_data);
    transmitString("hardStopRight\n\r");
}

//  hardStopLeft
//  Stops the left wheel without the compensator
//  inputs:     none
//  outputs:    none
void hardStopLeft()
{
    setWheelDutyCycle(&left_wheel_data, 0);
    wheelUpdateMove(&left_wheel_data);
    rover_state = ABNORMAL;
    move_end_cond = NONE;
    left_wheel_data.des_rpm = 0;
    clearStateVariables(&left_wheel_data);
    transmitString("hardStopLeft\n\r");
}

//  disableRover
//  Disables the rover by setting the SLP pin low
//  inputs:     none
//  outputs:    none
void disableRover()
{
    disableWheel(&right_wheel_data);
    disableWheel(&left_wheel_data);
}

//  enableRover
//  Enables the rover by setting the SLP pin high
//  inputs:     none
//  outputs:    none
void enableRover()
{
    enableWheel(&right_wheel_data);
    enableWheel(&left_wheel_data);
}

//  checkEndCond
//  Determines if either a time condition or rotation condition was
//  met and stops the rover if it has.
//  inputs:     none
//  outputs:    none
void checkEndCond()
{
    if(rover_state != ROVER_STOPPED)
    {
        switch(move_end_cond)
        {
        case TIME:      if(timeConditionMet())
                        {
                            resetTimeTracker();
                            if(right_wheel_data.enable_compensator || left_wheel_data.enable_compensator)
                                stopRover();
                            else
                                hardStop();
                        }
                        break;
        case ROTATION:  if(rotationConditionMet())
                        {
                            resetRotationTracker();
                            hardStop();
                        }
                        break;
        default: break;
        }
    }
}
//  compensator
//  Implements a compensator for the RSLK wheels. Using the
//  compensator provides RPM outputs that more closely match
//  the desired RPM. However, the RPM output is not as consistent
//  as without the compensator. There are move functions that use
//  compensator or not. Must be called on every loop iteration.
//  inputs:     none
//  outputs:    none
void compensator()
{
    if(right_wheel_data.enable_compensator)
    {
        if(right_wheel_data.compensator_count >= COMP_SAMPLING_PERIOD)
        {
            uint32_t current_rpm = calcCurrentRPM(&right_wheel_data);       //in deci RPM
            int32_t error = (right_wheel_data.des_rpm - current_rpm) * COMP_SCALAR;        //find error and scale

            if(!right_wheel_data.abnormal_wheel_stop)
            {
                //update state variables
                right_wheel_data.p[0] = error / RIGHT_WHEEL_COMP_GAIN  + right_wheel_data.p[1];
                if(right_wheel_data.des_rpm == 0 && error == 0)
                    right_wheel_data.p[1] = 0;
                else
                    right_wheel_data.p[1] = right_wheel_data.p[0];
                //set output
                double dutyCycle = (double)(right_wheel_data.p[0]) / COMP_SCALAR / 100;
                if(dutyCycle < 0)
                {
                    dutyCycle = 0;
                    right_wheel_data.p[1] = 0;
                }

                setWheelDutyCycle(&right_wheel_data, dutyCycle);
                wheelUpdateMove(&right_wheel_data);
            }

            right_wheel_data.compensator_count = 0;

            //if after an abnormal wheel stop the desired RPM was set by the user again
            if(right_wheel_data.abnormal_wheel_stop && right_wheel_data.des_rpm != 0)
            {
                right_wheel_data.abnormal_wheel_stop = false;
            }
            else if(!right_wheel_data.abnormal_wheel_stop && right_wheel_data.p[0] > COMP_OVERSHOOT_MAX)
            { //rover is stuck for some reason, or is powered off, or system is unstable - stop trying
                hardStopRight();
                right_wheel_data.abnormal_wheel_stop = true;
            }
        }
    }

    if(left_wheel_data.enable_compensator)
    {
        if(left_wheel_data.compensator_count >= COMP_SAMPLING_PERIOD)
        {
            uint32_t current_rpm = calcCurrentRPM(&left_wheel_data);       //in deci RPM
            int32_t error = (left_wheel_data.des_rpm - current_rpm) * COMP_SCALAR;        //find error and scale

            if(!left_wheel_data.abnormal_wheel_stop)
            {
                //update state variables
                left_wheel_data.p[0] = error / LEFT_WHEEL_COMP_GAIN  + left_wheel_data.p[1];

                if(left_wheel_data.des_rpm == 0 && error == 0)
                    left_wheel_data.p[1] = 0;
                else
                    left_wheel_data.p[1] = left_wheel_data.p[0];
                //set output
                double dutyCycle = (double)(left_wheel_data.p[0]) / COMP_SCALAR / 100;
                if(dutyCycle < 0)
                {
                    dutyCycle = 0;
                    left_wheel_data.p[1] = 0;
                }

                setWheelDutyCycle(&left_wheel_data, dutyCycle);
                wheelUpdateMove(&left_wheel_data);
            }

            left_wheel_data.compensator_count = 0;

            //if after an abnormal wheel stop the desired RPM was set by the user again
            if(left_wheel_data.abnormal_wheel_stop && left_wheel_data.des_rpm != 0)
            {
                left_wheel_data.abnormal_wheel_stop = false;
            }
            else if(!left_wheel_data.abnormal_wheel_stop && left_wheel_data.p[0] > COMP_OVERSHOOT_MAX)
            { //rover is stuck for some reason, or is powered off, or system is unstable - stop trying
                hardStopLeft();
                left_wheel_data.abnormal_wheel_stop = true;
            }
        }
    }
}
//  timeConditionMet
//  Determines if the time condition is met. Used for the 'forTime' move functions
//  inputs:     none
//  outputs:    bool indicating if time condition was met
bool timeConditionMet()
{
    bool time_cond = false;
    if((move_timer.count / T32_TICK_TO_MS) > move_timer.target_count)
        time_cond = true;

    return time_cond;
}

//  resetTimeTracker
//  Resets the move_timer struct to a null state
//  inputs:     none
//  outputs:    none
void resetTimeTracker()
{
    move_timer.count = 0;
    move_timer.target_count = 0;
}

//  resetRotationTracker
//  Resets the rover_rotation_tracker struct to a null state
//  inputs:     none
//  outputs:    none
void resetRotationTracker()
{
    rover_rotation_tracker.current_rotation = 0;
    rover_rotation_tracker.desired_degree = 0;
    right_wheel_data.wheel_rotation_tracker.ticks = 0;
    left_wheel_data.wheel_rotation_tracker.ticks = 0;
}

//  rotationConditionMet
//  Determines if a wheel rotation condition was met. Used for rotation and distance move functions
//  inputs:     none
//  outputs:    bool indicating if rotation condition was met
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

//  currentRoverRotation
//  Finds the current rotation of each wheel in degrees from when the measurement was started
//  inputs:     none
//  outputs:    none
void currentRoverRotation()
{
    //averaging the rotation counts on both wheels because they should be the same
    rover_rotation_tracker.current_rotation = (right_wheel_data.wheel_rotation_tracker.ticks +
            left_wheel_data.wheel_rotation_tracker.ticks) / 2;
}

//  updateRoverState
//  Meant to be called on each loop iteration. Updates the wheel states of the rover.
//  It will update the rover state if that is enabled by the user.
//  inputs:     none
//  outputs:    none
void updateRoverState()
{
    updateWheelState(&right_wheel_data);
    updateWheelState(&left_wheel_data);

    if(REAL_TIME_ROVER_STATE_TRACKING)
    {
        if(right_wheel_data.wheel_state == FORWARD && left_wheel_data.wheel_state == FORWARD)
        {
            rover_state = MOVING_FORWARD;
        }
        else if(right_wheel_data.wheel_state == FORWARD && left_wheel_data.wheel_state == BACKWARD)
        {
            rover_state = ROTATING_LEFT;
        }
        else if(right_wheel_data.wheel_state == BACKWARD && left_wheel_data.wheel_state == BACKWARD)
        {
            rover_state = MOVING_BACKWARD;
        }
        else if(right_wheel_data.wheel_state == BACKWARD && left_wheel_data.wheel_state == FORWARD)
        {
            rover_state = ROTATING_RIGHT;
        }
        else
        {
            rover_state = ABNORMAL;
        }
    }
}

//  clearEncBInterrupt
//  Clears the interrupts on the motor encoder port
//  inputs:     none
//  outputs:    none
void clearEncBInterrupt()
{
    GPIO_clearInterruptFlag(right_wheel_data.wheel_encb_pin.portNum, right_wheel_data.wheel_encb_pin.pinNum);
    GPIO_clearInterruptFlag(left_wheel_data.wheel_encb_pin.portNum, left_wheel_data.wheel_encb_pin.pinNum);
}

//  initRSLKRover
//  Initializes RSLK Rover structs
//  inputs:     _sys_clk in Hz
//  outputs:    none
void initRSLKRover(uint32_t _sys_clk)
{
    rover_state = ROVER_STOPPED;
    move_end_cond = NONE;
    move_timer.count = 0;
    move_timer.target_count = 0;
    t32_data.sys_clk_freq = _sys_clk;
    t32_data.timer_period = T32_1_PERIOD;
    t32_data.count_max = _sys_clk * T32_1_PERIOD;   //T32 will always have a period of T32_1_PERIOD no matter the system clock
    rover_rotation_tracker.desired_degree = 0;
    rover_rotation_tracker.current_rotation = 0;


    initRightRSLKWheel(&right_wheel_data, t32_data.sys_clk_freq, t32_data.count_max);
    initLeftRSLKWheel(&left_wheel_data, t32_data.sys_clk_freq, t32_data.count_max);
    Interrupt_enableMaster();
}

//  initRSLKTimer32
//  Initializes the Timer32 that the RSLK will use
//  inputs:     none
//  outputs:    none
void initRSLKTimer32()
{
    //T32 operates off of the master clock
    Timer32_initModule(RSLK_TIMER32_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_setCount(RSLK_TIMER32_BASE, t32_data.count_max);
    Timer32_enableInterrupt(RSLK_TIMER32_BASE);
    Interrupt_enableInterrupt(INT_T32_INT2);
    Timer32_startTimer(RSLK_TIMER32_BASE, CONTINUOUS);
}

//  rawConvertRPMToDutyCycle
//  Maps an inputted RPM to a duty cycle based on a rough approximation
//  This function is used to set the motor duty cycle for all
//  non-compensated move functions.
//  inputs:     _rpm to convert to duty cycle
//  outputs:    double that indicates duty cycle in fractional form
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

//  rotationDegConversion
//  Converts an overall rover rotation to
//  an individual wheel rotation
//  inputs:     _deg to convert to individual wheel rotation
//  outputs:    uint32_t indicates individual wheel rotation
uint32_t rotationDegConversion(uint32_t _deg)
{
    uint32_t conv_deg;
    conv_deg = _deg * ROVER_WIDTH_TO_WHEEL_DIAM_RATIO;
    return conv_deg;
}

//  distanceDegConversion
//  Converts an overall desired distance for the RSLK
//  to travel to an individual wheel rotation.
//  inputs:     _mm to convert to number of wheel turns
//  outputs:    uint32_t indicates wheel turns in degrees
uint32_t distanceDegConversion(uint32_t _mm)
{
    uint32_t conv_deg;
    conv_deg = DEG_PER_RAD * _mm / WHEEL_RADIUS;
    return conv_deg;
}

//  enableCompensator
//  Enables the compensator
//  inputs:     none
//  outputs:    none
void enableCompensator()
{
    enableWheelCompensator(&right_wheel_data);
    enableWheelCompensator(&left_wheel_data);
}

//  disableCompensator
//  Disables the compensator
//  inputs:     none
//  outputs:    none
void disableCompensator()
{
    disableWheelCompensator(&right_wheel_data);
    disableWheelCompensator(&left_wheel_data);
}

//COMPENSATED MOVE FUNCTIONS BELOW//////////////////////////

//  moveForwardForTimeComp
//  robot moves forward for a specified amount of time
//  inputs:     _rpm indicates robot speed
//              milliSecs indicates amount of time
//  outputs:    none
void moveForwardForTimeComp(uint32_t _rpm, uint32_t milliSecs)
{
    enableCompensator();
    //nominal _rpm unit is deci rpm
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;

    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    setWheelDirForward(&right_wheel_data);
    setWheelDirForward(&left_wheel_data);
    rover_state = MOVING_FORWARD;
}

//  moveBackwardForTimeComp
//  robot moves backward for a specified amount of time
//  inputs:     _rpm indicates robot speed
//              milliSecs indicates amount of time
//  outputs:    none
void moveBackwardForTimeComp(uint32_t _rpm, uint32_t milliSecs)
{
    enableCompensator();
    //nominal _rpm unit is deci rpm
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;

    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    setWheelDirBackward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    rover_state = MOVING_BACKWARD;
}

//  rotateRightForTimeComp
//  robot rotates to the right for a specified amount of time
//  inputs:     _rpm indicates robot speed
//              milliSecs indicates amount of time
//  outputs:    none
void rotateRightForTimeComp(uint32_t _rpm, uint32_t milliSecs)
{
    enableCompensator();
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;

    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    setWheelDirForward(&left_wheel_data);
    setWheelDirBackward(&right_wheel_data);
    rover_state = ROTATING_RIGHT;
}

//  rotateLeftForTimeComp
//  robot rotates to the left for a specified amount of time
//  inputs:     _rpm indicates robot speed
//              milliSecs indicates amount of time
//  outputs:    none
void rotateLeftForTimeComp(uint32_t _rpm, uint32_t milliSecs)
{
    enableCompensator();
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;

    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    setWheelDirForward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    rover_state = ROTATING_LEFT;
}

//  moveForwardIndefinitelyComp
//  robot moves forward indefinitely
//  inputs:     _rpm indicates robot speed
//  outputs:    none
void moveForwardIndefinitelyComp(uint32_t _rpm)
{
    enableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDirForward(&right_wheel_data);
    setWheelDirForward(&left_wheel_data);
    rover_state = MOVING_FORWARD;
    move_end_cond = NONE;
}

//  moveBackwardIndefinitelyComp
//  robot moves backward indefinitely
//  inputs:     _rpm indicates robot speed
//  outputs:    none
void moveBackwardIndefinitelyComp(uint32_t _rpm)
{
    enableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    setWheelDirBackward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    rover_state = MOVING_BACKWARD;
    move_end_cond = NONE;
}

//  rotateRightIndefinitelyComp
//  robot rotates right indefinitely
//  inputs:     _rpm indicates robot speed
//  outputs:    none
void rotateRightIndefinitelyComp(uint32_t _rpm)
{
    enableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    setWheelDirForward(&left_wheel_data);
    setWheelDirBackward(&right_wheel_data);
    rover_state = ROTATING_RIGHT;
    move_end_cond = NONE;
}

//  rotateLeftIndefinitelyComp
//  robot rotates left indefinitely
//  inputs:     _rpm indicates robot speed
//  outputs:    none
void rotateLeftIndefinitelyComp(uint32_t _rpm)
{
    enableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    setWheelDirForward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    rover_state = ROTATING_LEFT;
    move_end_cond = NONE;
}

//  rotateRightForDegreesComp
//  robot rotates right for a specified amount of degrees
//  inputs:     _rpm indicates robot speed
//              degrees indicate number of degrees to turn
//  outputs:    none
void rotateRightForDegreesComp(uint32_t _rpm, uint32_t degrees)
{
    enableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    rover_rotation_tracker.desired_degree = rotationDegConversion(degrees);
    rover_rotation_tracker.current_rotation = 0;
    setWheelDirForward(&left_wheel_data);
    setWheelDirBackward(&right_wheel_data);
    rover_state = ROTATING_RIGHT;
    move_end_cond = ROTATION;
}

//  rotateLeftForDegreesComp
//  robot rotates left for a specified amount of degrees
//  inputs:     _rpm indicates robot speed
//              degrees indicate number of degrees to turn
//  outputs:    none
void rotateLeftForDegreesComp(uint32_t _rpm, uint32_t degrees)
{
    enableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    rover_rotation_tracker.desired_degree = rotationDegConversion(degrees);
    rover_rotation_tracker.current_rotation = 0;
    setWheelDirForward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    rover_state = ROTATING_LEFT;
    move_end_cond = ROTATION;
}

//  moveForwardForDistanceComp
//  robot moves forward a specified distance
//  inputs:     _rpm indicates robot speed
//              mm indicates distance to travel
//  outputs:    none
void moveForwardForDistanceComp(uint32_t _rpm, uint32_t mm)
{
    enableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    rover_rotation_tracker.desired_degree = distanceDegConversion(mm);
    rover_rotation_tracker.current_rotation = 0;
    setWheelDirForward(&right_wheel_data);
    setWheelDirForward(&left_wheel_data);
    rover_state = MOVING_FORWARD;
    move_end_cond = ROTATION;
}

//  moveBackwardForDistanceComp
//  robot moves backward a specified distance
//  inputs:     _rpm indicates robot speed
//              mm indicates distance to travel
//  outputs:    none
void moveBackwardForDistanceComp(uint32_t _rpm, uint32_t mm)
{
    enableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;

    rover_rotation_tracker.desired_degree = distanceDegConversion(mm);
    rover_rotation_tracker.current_rotation = 0;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDirBackward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    rover_state = MOVING_BACKWARD;
    move_end_cond = ROTATION;
}

/////NON COMPENSATED MOVE FUNCTIONS BELOW

//  moveForwardForTime
//  robot moves forward for a specified amount of time
//  no compensation
//  inputs:     _rpm indicates robot speed
//              milliSecs indicates amount of time
//  outputs:    none
void moveForwardForTime(uint32_t _rpm, uint32_t milliSecs)
{
    disableCompensator();
    //nominal _rpm unit is deci rpm
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&right_wheel_data);
    setWheelDirForward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = MOVING_FORWARD;
}

//  moveBackwardForTime
//  robot moves backward for a specified amount of time
//  no compensation
//  inputs:     _rpm indicates robot speed
//              milliSecs indicates amount of time
//  outputs:    none
void moveBackwardForTime(uint32_t _rpm, uint32_t milliSecs)
{
    disableCompensator();
    //nominal _rpm unit is deci rpm
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirBackward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = MOVING_BACKWARD;
}

//  rotateRightForTime
//  robot rotates to the right for a specified amount of time
//  no compensation
//  inputs:     _rpm indicates robot speed
//              milliSecs indicates amount of time
//  outputs:    none
void rotateRightForTime(uint32_t _rpm, uint32_t milliSecs)
{
    disableCompensator();
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&left_wheel_data);
    setWheelDirBackward(&right_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = ROTATING_RIGHT;
}

//  rotateLeftForTime
//  robot rotates to the left for a specified amount of time
//  no compensation
//  inputs:     _rpm indicates robot speed
//              milliSecs indicates amount of time
//  outputs:    none
void rotateLeftForTime(uint32_t _rpm, uint32_t milliSecs)
{
    disableCompensator();
    move_timer.target_count = milliSecs;
    move_timer.count = 0;
    move_end_cond = TIME;
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
    double dutyCycle = rawConvertRPMToDutyCycle(_rpm);
    setWheelDutyCycle(&right_wheel_data, dutyCycle);
    setWheelDutyCycle(&left_wheel_data, dutyCycle);
    setWheelDirForward(&right_wheel_data);
    setWheelDirBackward(&left_wheel_data);
    wheelUpdateMove(&right_wheel_data);
    wheelUpdateMove(&left_wheel_data);
    rover_state = ROTATING_LEFT;
}

//  moveForwardIndefinitely
//  robot moves forward indefinitely
//  no compensation
//  inputs:     _rpm indicates robot speed
//  outputs:    none
void moveForwardIndefinitely(uint32_t _rpm)
{
    disableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
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

//  moveBackwardIndefinitely
//  robot moves backward indefinitely
//  no compensation
//  inputs:     _rpm indicates robot speed
//  outputs:    none
void moveBackwardIndefinitely(uint32_t _rpm)
{
    disableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
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

//  rotateRightIndefinitely
//  robot rotates to the right indefinitely
//  no compensation
//  inputs:     _rpm indicates robot speed
//  outputs:    none
void rotateRightIndefinitely(uint32_t _rpm)
{
    disableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
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

//  rotateLeftIndefinitely
//  robot rotates to the left indefinitely
//  no compensation
//  inputs:     _rpm indicates robot speed
//  outputs:    none
void rotateLeftIndefinitely(uint32_t _rpm)
{
    disableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
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

//  rotateRightForDegrees
//  robot rotates right for a specified amount of degrees
//  no compensation
//  inputs:     _rpm indicates robot speed
//              degrees indicate number of degrees to turn
//  outputs:    none
void rotateRightForDegrees(uint32_t _rpm, uint32_t degrees)
{
    disableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
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

//  rotateLeftForDegrees
//  robot rotates left for a specified amount of degrees
//  no compensation
//  inputs:     _rpm indicates robot speed
//              degrees indicate number of degrees to turn
//  outputs:    none
void rotateLeftForDegrees(uint32_t _rpm, uint32_t degrees)
{
    disableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
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

//  moveForwardForDistance
//  robot moves forward a specified distance
//  no compensation
//  inputs:     _rpm indicates robot speed
//              mm indicates distance to travel
//  outputs:    none
void moveForwardForDistance(uint32_t _rpm, uint32_t mm)
{
    disableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
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

//  moveBackwardForDistance
//  robot moves backward a specified distance
//  no compensation
//  inputs:     _rpm indicates robot speed
//              mm indicates distance to travel
//  outputs:    none
void moveBackwardForDistance(uint32_t _rpm, uint32_t mm)
{
    disableCompensator();
    right_wheel_data.des_rpm = _rpm;
    left_wheel_data.des_rpm = _rpm;
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

