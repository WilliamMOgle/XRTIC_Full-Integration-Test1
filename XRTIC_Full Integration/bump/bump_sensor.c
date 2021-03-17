/*
 * bump_sensor.c
 *
 *  Created on: Mar 11, 2021
 *      Author: onemo
 */

#include "bump_sensor.h"

void initBumpSensors()
{
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP0);
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP1);
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP2);
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP3);
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP4);
    GPIO_setAsInputPinWithPullUpResistor(BUMP_PORT, BUMP5);

}
uint8_t getBumpSensorInput(uint_fast16_t bumpPin)
{
    return GPIO_getInputPinValue(BUMP_PORT, bumpPin);
}

bool bumpSensorPressed(uint_fast16_t bumpPin)
{
    if(GPIO_getInputPinValue(BUMP_PORT, bumpPin))
        return false;
    return true;
}
