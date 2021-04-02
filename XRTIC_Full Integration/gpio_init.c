/*
 * initialize.c
 *
 *  Created on: Mar 18, 2021
 *      Author: wills
 */

#include <gpio_init.h>


void initialize_LaunchpadLEDs()
{
    initialize_LaunchpadLED1();
    initialize_LaunchpadLED2_red();
    initialize_LaunchpadLED2_blue();
    initialize_LaunchpadLED2_green();
}

void initialize_LaunchpadLED1()
{
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
}
void turnOn_LaunchpadLED1()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
}
void turnOff_LaunchpadLED1()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
}
void toggle_LaunchpadLED1()
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void initialize_LaunchpadLED2_red()
{
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
}
void turnOn_LaunchpadLED2_red()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
}
void turnOff_LaunchpadLED2_red()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
}
void toggle_LaunchpadLED2_red()
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);
}

void initialize_LaunchpadLED2_green()
{
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
}
void turnOn_LaunchpadLED2_green()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
}
void turnOff_LaunchpadLED2_green()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
}
void toggle_LaunchpadLED2_green()
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
}

void initialize_LaunchpadLED2_blue()
{
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
}
void turnOn_LaunchpadLED2_blue()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
}
void turnOff_LaunchpadLED2_blue()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
}
void toggle_LaunchpadLED2_blue()
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);
}
