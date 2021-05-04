/*
 * bump_sensor.c
 *
 *  Created on: Mar 11, 2021
 *      Author: onemo
 */

#include "bump_sensor.h"

//  initBumpsensors
//  initializes the bump sensors for use
//  inputs:     none
//  outputs:    none
void initBumpSensors()
{
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP0);
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP1);
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP2);
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP3);
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP4);
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP5);

}

//  getBumpSensorInput
//  returns the state of a given bump sensor
//  inputs:     bumpPin indicates pin number of bump sensor
//  outputs:    uint8_t indicates state of bump sensor
uint8_t getBumpSensorInput(uint_fast16_t bumpPin)
{
    return GPIO_getInputPinValue(BUMP_PORT, bumpPin);
}

//  bumpSensorPressed
//  returns whether a bump sensor is pressed
//  inputs:     bumpPin indicates pin number of bump sensor
//  outputs:    bool indicates state of bump sensor
bool bumpSensorPressed(uint_fast16_t bumpPin)
{
    if(GPIO_getInputPinValue(BUMP_PORT, bumpPin))
        return false;
    return true;
}
