/*
 * main.h
 *
 *  Created on: Mar 18, 2021
 *      Author: wills
 */

#ifndef MAIN_H_
#define MAIN_H_


//----INCLUDES---------
//subsystems
#include "mqtt.h"
#include "nfc.h"
#include "servo180.h"
#include "driver_7seg.h"
#include "bump_sensor.h"
//helper
#include <gpio_init.h>
#include "jsonParser.h"

//ROVER INCLUDES AND MQTT INCLUDES
// Standard includes
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <driverlib.h>

//IR sensor
#include "msp.h"
#include "Clock.h"
#include "ir/I2CB1.h"
#include "ir/CortexM.h"
#include "ir/LPF.h"
#include "ir/opt3101.h"
#include "ir/LaunchPad.h"

//UART INCLUDES
#include "uart_HAL.h"

//Rover
#include "RSLK_Wheel.h"
#include "Timer_HAL.h"
#include "RSLK_Rover.h"

//---------DEFINES----------
//system enables
#define NFC_ENABLE false
#define ROVER_ENABLE false
#define BUMP_ENABLE false
#define ROBONAV_ENABLE false
#define GRIPPER_ENABLE false
#define MQTT_ENABLE false

//ROVER DEFINES
#define SYS_CLK 48000000
#define ROVER_SPEED 2500

//ROBONAV DEFINES

#define MIN_DISTANCE 100;
#define INVALID_DISTANCE 65535;
#define BUMP_ROVER_RPM 500;
#define BUMP_ROVER_TURN_DEGREES 360;
#define BUMP_ROVER_BACKWARDS_MM 100;


typedef enum
{
    OPENING,
    CLOSING,
    HOLD
}Gripper_State;

typedef enum
{
    SPEED_INCREASE,
    SPEED_DECREASE,
    SPEED_HOLD
}Speed_Set;

//------GLOBALS---------
//rover
Speed_Set speed_state;
uint16_t speed;
//IR sensor
uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec

//FUNCTION PROTOTYPES
void roboNav();

#endif /* MAIN_H_ */
