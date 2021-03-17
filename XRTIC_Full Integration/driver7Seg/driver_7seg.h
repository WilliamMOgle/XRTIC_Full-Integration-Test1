/*
 * 7seg_driver.h
 *
 *  Created on: Mar 11, 2021
 *      Author: onemo
 */


#ifndef DRIVER_7SEG_H_
#define DRIVER_7SEG_H_


#include "driver_7seg_config.h"
#include <driverlib.h>

void initializeOutputs();
void segmentWrite(char c);
void writeOnA();
void writeOffA();
void writeOnB();
void writeOffB();
void writeOnC();
void writeOffC();
void writeOnD();
void writeOffD();
void writeOnE();
void writeOffE();
void writeOnF();
void writeOffF();
void writeOnG();

void writeOnAll();
void writeOffAll();
void writeOnDP();
void writeOffDP();



#endif /* DRIVER_7SEG_H_ */
