/*
 * servo180.c
 *
 *  Created on: Feb 3, 2021
 *      Author: wills
 */

#include <servo180.h>

//  servo180Init
//  initializes servo given a fully defined Servo180 struct
//  inputs:     servoSettings point to Servo180 struct
//  outputs:    none
void servo180Init(Servo180 *servoSettings)
{
    setPWM(&servoSettings->pwm_settings);
}
//  servo180InitArgs
//  initializes servo with user-inputted arguments
//  inputs:     servoSettings point to Servo180 struct
//              _sys_clk in Hz
//              _closed_deg in degrees
//              _open_deg in degrees
//              _port indicates port number
//              _pin indicates pin number
//  outputs:    none
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

//  moveServoToDegree
//  moves the servo horn to a specific degree
//  inputs:     degree indicates degree number to turn to
//              servoSettings point to Servo180 struct
//  outputs:    bool indicating successful move
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

//  convertDegToDuty
//  helper function that converts a degree to a duty cycle percent via a linear mapping function
//  inputs:     degree to convert to duty cycle
//              servoSettings point to Servo180 struct
//  outputs:    double indicating duty cycle
double convertDegToDuty(double degree, Servo180 *servoSettings)
{
    double dutyCycle = ((degree - ABSOLUTE_LOWER_DEG_BOUND) * (ABSOLUTE_UPPER_DUTY_BOUND - ABSOLUTE_LOWER_DUTY_BOUND)
            / (ABSOLUTE_UPPER_DEG_BOUND - ABSOLUTE_LOWER_DEG_BOUND) + ABSOLUTE_LOWER_DUTY_BOUND);
    return dutyCycle;
}

//  getDegree
//  returns the current degree of the servo
//  inputs:     servoSettings point to Servo180 struct
//  outputs:    double indicating number of degrees
double getDegree(Servo180 *servoSettings)
{
    return servoSettings->degree;
}

//  setOpenDegree
//  sets the open degree
//  inputs:     degree indicates degree number to turn to
//              servoSettings point to Servo180 struct
//  outputs:    none
void setOpenDegree(double degree, Servo180 *servoSettings)
{
    servoSettings->openDeg = degree;
}

//  setClosedDegree
//  sets the closed degree
//  inputs:     degree indicates degree number to turn to
//              servoSettings point to Servo180 struct
//  outputs:    none
void setClosedDegree(double degree, Servo180 *servoSettings)
{
    servoSettings->closedDeg = degree;
}

//  openServo
//  opens the servo as defined by 'openDeg'
//  inputs:     servoSettings point to Servo180 struct
//  outputs:    none
void openServo(Servo180 *servoSettings)
{
    moveServoToDegree(servoSettings->openDeg, servoSettings);
}

//  closeServo
//  closes the servo as defined by 'closedDeg'
//  inputs:     servoSettings point to Servo180 struct
//  outputs:    none
void closeServo(Servo180 *servoSettings)
{
    moveServoToDegree(servoSettings->closedDeg, servoSettings);
}

//  toggleOpenClose
//  toggles between open and closed, as defined by 'closedDeg' and 'openDeg'
//  if servo is not currently open or closed, the servo is moved to the open position
//  inputs:     servoSettings point to Servo180 struct
//  outputs:    none
void toggleOpenClose(Servo180 *servoSettings)
{
    if(servoSettings->degree == servoSettings->closedDeg)
        openServo(servoSettings);
    else if(servoSettings->degree == servoSettings->openDeg)
        closeServo(servoSettings);
    else
        openServo(servoSettings);
}
