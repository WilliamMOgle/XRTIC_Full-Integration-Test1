/*
 * servo180.h
 *
 *  Created on: Feb 3, 2021
 *      Author: wills
 */

#ifndef SERVO180_H_
#define SERVO180_H_

//ASSUMPTIONS
//frequency set at 50Hz
//servo is designed to move from 0-180 degrees

//INCLUDES
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <pwm_HAL.h>

//DEFINES
#define ABSOLUTE_LOWER_DUTY_BOUND   0.025 //2.5% DUTY CYCLE
#define ABSOLUTE_UPPER_DUTY_BOUND   0.125 //12.5% DUTY CYCLE
#define ABSOLUTE_LOWER_DEG_BOUND    0
#define ABSOLUTE_UPPER_DEG_BOUND    180
#define FREQ_HZ                     50
#define DEF_OUT_PORT                GPIO_PORT_P2
#define DEF_OUT_PIN                 GPIO_PIN4
#define DEF_CLOSED_DEG              135
#define DEF_OPEN_DEG                45
#define DEF_SYS_CLK                 48000000

//STRUCTS
typedef struct{
    //User Variables
    uint32_t sys_clk;           //system clock speed
    double openDeg;             //degree that is defined as "gripper open"
    double closedDeg;           //degree that is defined as "gripper closed"

    //Internal Variables
    double degree;              //current degree of servo (degrees)
    PWM_Params pwm_settings;    //PWM control struct
}Servo180;

//FUNCTION PROTOTYPES
void servo180Init(Servo180 *servoSettings);
void servo180InitArgs(Servo180 *, uint32_t _sys_clk, double closed_deg, double open_deg, uint32_t _port, uint32_t _pin);
bool moveServoToDegree(double degree, Servo180 *servoSettings);
double convertDegToDuty(double degree, Servo180 *servoSettings);
void setOpenDegree(double degree, Servo180 *servoSettings);
void setClosedDegree(double degree, Servo180 *servoSettings);
void openServo(Servo180 *servoSettings);
void closeServo(Servo180 *servoSettings);
void toggleOpenClose(Servo180*);
double getDegree(Servo180 *servoSettings);

#endif /* SERVO180_H_ */
