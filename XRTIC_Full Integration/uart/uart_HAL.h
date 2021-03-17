/*
 * uart_HAL.h
 *
 *  Created on: Mar 10, 2021
 *      Author: onemo
 */

#ifndef UART_HAL_H_
#define UART_HAL_H_

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "uart_config.h"
/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

void initUART();
void initMCU();
void transmitChar(uint8_t);
void transmitString(uint8_t*);
void transmitInt(int);
char transmitIntHelper(int i);




#endif /* UART_HAL_H_ */
