/*
 * servo180.c
 *
 *  Created on: Feb 3, 2021
 *      Author: wills
 */

#include <servo180.h>

//initializes servo given a fully defined Servo180 struct
void servo180Init(Servo180 *servoSettings)
{
    setPWM(&servoSettings->pwm_settings);
}

//initializes servo with user-inputted arguments
void servo180InitArgs(Servo180 *servoSettings, uint32_t _sys_clk, double _closed_deg,
                      double _open_deg, uint32_t _port, uint32_t _pin)
{
    servoSettings->degree = -1;
    servoSettings->sys_clk = _sys_clk;
    servoSettings->closedDeg = _closed_deg;
    servoSettings->openDeg = _open_deg;

    servoSettings->pwm_settings.sys_clk = servoSettings->sys_clk;
    servoSettings->pwm_settings.freq = FREQ_HZ;
    servoSettings->pwm_settings.dutyCycle = 0;
    servoSettings->pwm_settings.output_pin.portNum = _port;
    servoSettings->pwm_settings.output_pin.pinNum = _pin;

    servo180Init(servoSettings);
}

//moves the servo horn to a specific degree
bool moveServoToDegree(double degree, Servo180 *servoSettings)
{
    double dutyCycle = convertDegToDuty(degree, servoSettings);
    bool goodMove = false;
    if(dutyCycle >= ABSOLUTE_LOWER_DUTY_BOUND && dutyCycle <= ABSOLUTE_UPPER_DUTY_BOUND)
    {
        goodMove = updateDutyCycle(dutyCycle, &servoSettings->pwm_settings);
        generatePWM(&servoSettings->pwm_settings);
    }

    if(goodMove)
        servoSettings->degree = degree;

    return goodMove;

}

//helper function that converts a degree to a duty cycle percent via a linear mapping function
double convertDegToDuty(double degree, Servo180 *servoSettings)
{
    double dutyCycle = ((degree - ABSOLUTE_LOWER_DEG_BOUND) * (ABSOLUTE_UPPER_DUTY_BOUND - ABSOLUTE_LOWER_DUTY_BOUND)
            / (ABSOLUTE_UPPER_DEG_BOUND - ABSOLUTE_LOWER_DEG_BOUND) + ABSOLUTE_LOWER_DUTY_BOUND);
    return dutyCycle;
}

//returns the current degree of the servo
double getDegree(Servo180 *servoSettings)
{
    return servoSettings->degree;
}

//sets the open degree
void setOpenDegree(double degree, Servo180 *servoSettings)
{
    servoSettings->openDeg = degree;
}

//sets the closed degree
void setClosedDegree(double degree, Servo180 *servoSettings)
{
    servoSettings->closedDeg = degree;
}

//opens the servo as defined by 'openDeg'
void openServo(Servo180 *servoSettings)
{
    moveServoToDegree(servoSettings->openDeg, servoSettings);
}

//closes the servo as defined by 'closedDeg'
void closeServo(Servo180 *servoSettings)
{
    moveServoToDegree(servoSettings->closedDeg, servoSettings);
}

//toggles between open and closed, as defined by 'closedDeg' and 'openDeg'
//if servo is not currently open or closed, the servo is moved to the open position
void toggleOpenClose(Servo180 *servoSettings)
{
    if(servoSettings->degree == servoSettings->closedDeg)
        openServo(servoSettings);
    else if(servoSettings->degree == servoSettings->openDeg)
        closeServo(servoSettings);
    else
        openServo(servoSettings);
}
