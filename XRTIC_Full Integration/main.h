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
#define NFC_ENABLE 1
#define ROVER_ENABLE 1
#define BUMP_ENABLE 1
#define IR_ENABLE 1
#define ROBONAV_ENABLE 1
#define GRIPPER_ENABLE 1
#define MQTT_ENABLE 1
#define SEG7_ENABLE 1
#define TELEM_ENABLE 1
#define REACT_ENABLE 1

//ROVER DEFINES
#define SYS_CLK 48000000
#define ROVER_SPEED 2500
#define NEG_REACTION_SPEED 2000

//ROBONAV DEFINES

#define MIN_DISTANCE 100;
#define INVALID_DISTANCE 65535;
#define BUMP_ROVER_RPM 500;
#define BUMP_ROVER_TURN_DEGREES 360;
#define BUMP_ROVER_BACKWARDS_MM 100;

#define MAX_MESSAGE_SIZE 60;

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

typedef enum
{
    POSITIVE_TAG,
    NEGATIVE_TAG,
    NO_TAG
}Tag_Type;

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
uint16_t g_ui16BytesReceived;// Number of bytes received from the host
//nfc
tNfcState eTempNFCState;
tNfcState eCurrentNFCState;
t_sNfcCEMode sCEMode;
t_sNfcP2PMode sP2PMode;
t_sNfcP2PCommBitrate sP2PBitrate;
t_sNfcRWMode sRWMode;
t_sNfcRWCommBitrate sRWBitrate;



//FUNCTION PROTOTYPES
void roboNav();
void roverInit();
void irInit();
void positiveReaction();
void negativeReaction();
Tag_Type nfc_tag_detect(bool*, uint8_t*);

#endif /* MAIN_H_ */
