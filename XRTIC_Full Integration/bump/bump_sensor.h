/*
 * bump_sensor.h
 *
 *  Created on: Mar 11, 2021
 *      Author: onemo
 */

#ifndef BUMP_SENSOR_H_
#define BUMP_SENSOR_H_

#include "bump_config.h"


void initBumpSensors();
uint8_t getBumpSensorInput(uint_fast16_t);
bool bumpSensorPressed(uint_fast16_t);

#endif /* BUMP_SENSOR_H_ */
