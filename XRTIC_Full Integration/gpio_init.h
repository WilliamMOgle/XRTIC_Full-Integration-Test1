/*
 * initialize.h
 *
 *  Created on: Mar 18, 2021
 *      Author: wills
 */

#ifndef GPIO_INIT_H_
#define GPIO_INIT_H_

#include <driverlib.h>

void initialize_LaunchpadLEDs();
void initialize_LaunchpadLED1();
void turnOn_LaunchpadLED1();
void turnOff_LaunchpadLED1();
void toggle_LaunchpadLED1();
void initialize_LaunchpadLED2_red();
void turnOn_LaunchpadLED2_red();
void turnOff_LaunchpadLED2_red();
void toggle_LaunchpadLED2_red();
void initialize_LaunchpadLED2_green();
void turnOn_LaunchpadLED2_green();
void turnOff_LaunchpadLED2_green();
void toggle_LaunchpadLED2_green();
void initialize_LaunchpadLED2_blue();
void turnOn_LaunchpadLED2_blue();
void turnOff_LaunchpadLED2_blue();
void toggle_LaunchpadLED2_blue();
void button_two_interrupt_init();




#endif /* GPIO_INIT_H_ */
