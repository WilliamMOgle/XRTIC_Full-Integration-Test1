/*
 * main.h
 *
 *  Created on: Mar 18, 2021
 *      Author: wills
 */

#ifndef MAIN_H_
#define MAIN_H_


#include "mqtt.h"
#include "nfc.h"
#include <driverlib.h>
#include <gpio_init.h>
#include "servo180.h"


/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>




#include "bump_sensor.h"
#include "jsonParser.h"

//ROVER INCLUDES AND MQTT INCLUDES
// Standard includes
#include <stdlib.h>
#include <string.h>




//UART INCLUDES
#include "uart_HAL.h"

#include "driver_7seg.h"

//ROVER INCLUDES
//#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "RSLK_Wheel.h"
#include "Timer_HAL.h"
#include "RSLK_Rover.h"
//#include <pwm_HAL.h>

//ROVER DEFINES
#define SYS_CLK 48000000


#endif /* MAIN_H_ */
