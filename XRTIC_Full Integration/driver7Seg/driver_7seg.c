/*
 * 7seg_driver.c
 *
 *  Created on: Mar 11, 2021
 *      Author: onemo
 */




#include "driver_7seg.h"

void writeOnA()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_A, GPIO_PIN_A);
}

void writeOffA()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_A, GPIO_PIN_A);
}
void writeOnB()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_B, GPIO_PIN_B);
}

void writeOffB()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_B, GPIO_PIN_B);
}
void writeOnC()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_C, GPIO_PIN_C);
}

void writeOffC()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_C, GPIO_PIN_C);
}

void writeOnD()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_D, GPIO_PIN_D);
}

void writeOffD()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_D, GPIO_PIN_D);
}

void writeOnE()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_E, GPIO_PIN_E);
}

void writeOffE()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_E, GPIO_PIN_E);
}

void writeOnF()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_F, GPIO_PIN_F);
}

void writeOffF()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_F, GPIO_PIN_F);
}

void writeOnG()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_G, GPIO_PIN_G);
}

void writeOffG()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_G, GPIO_PIN_G);
}

void writeOnDP()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_DP, GPIO_PIN_DP);
}

void writeOffDP()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_DP, GPIO_PIN_DP);
}

void writeOnAll()
{
    writeOnA();
    writeOnB();
    writeOnC();
    writeOnD();
    writeOnE();
    writeOnF();
    writeOnG();
    writeOnDP();
}

void writeOffAll()
{
    writeOffA();
    writeOffB();
    writeOffC();
    writeOffD();
    writeOffE();
    writeOffF();
    writeOffG();
    writeOffDP();
}

void initializeOutputs()
{
    GPIO_setAsOutputPin(GPIO_PORT_A, GPIO_PIN_A);
    GPIO_setAsOutputPin(GPIO_PORT_B, GPIO_PIN_B);
    GPIO_setAsOutputPin(GPIO_PORT_C, GPIO_PIN_C);
    GPIO_setAsOutputPin(GPIO_PORT_D, GPIO_PIN_D);
    GPIO_setAsOutputPin(GPIO_PORT_E, GPIO_PIN_E);
    GPIO_setAsOutputPin(GPIO_PORT_F, GPIO_PIN_F);
    GPIO_setAsOutputPin(GPIO_PORT_G, GPIO_PIN_G);
    GPIO_setAsOutputPin(GPIO_PORT_DP, GPIO_PIN_DP);
}

void segmentWrite(char c)
{
    writeOffAll();

    switch(c)
    {
        case '0':
            writeOnA();
            writeOnB();
            writeOnC();
            writeOnD();
            writeOnE();
            writeOnF();
            break;
        case '1':
            writeOnB();
            writeOnC();
            break;
        case '2':
            writeOnA();
            writeOnB();
            writeOnD();
            writeOnE();
            writeOnG();
            break;
        case '3':
            writeOnA();
            writeOnB();
            writeOnC();
            writeOnD();
            writeOnG();
            break;
        case '4':
            writeOnB();
            writeOnC();
            writeOnF();
            writeOnG();
            break;
        case '5':
            writeOnA();
            writeOnC();
            writeOnD();
            writeOnF();
            writeOnG();
            break;
        case '6':
            writeOnA();
            writeOnC();
            writeOnD();
            writeOnE();
            writeOnF();
            writeOnG();
            break;
        case '7':
            writeOnA();
            writeOnB();
            writeOnC();
            break;
        case '8':
            writeOnA();
            writeOnB();
            writeOnC();
            writeOnD();
            writeOnE();
            writeOnF();
            writeOnG();
            break;
        case '9':
            writeOnA();
            writeOnB();
            writeOnC();
            writeOnD();
            writeOnF();
            writeOnG();
            break;
        case 'a':
            writeOnA();
            writeOnB();
            writeOnC();
            writeOnE();
            writeOnF();
            writeOnG();
            break;
        case 'b':
            writeOnC();
            writeOnD();
            writeOnE();
            writeOnF();
            writeOnG();
            break;
        case 'c':
            writeOnA();
            writeOnD();
            writeOnE();
            writeOnF();
            break;
        case 'd':
            writeOnB();
            writeOnC();
            writeOnD();
            writeOnE();
            writeOnG();
            break;
        case 'e':
            writeOnA();
            writeOnD();
            writeOnE();
            writeOnF();
            writeOnG();
            break;
        case 'f':
            writeOnA();
            writeOnE();
            writeOnF();
            writeOnG();
            break;
        case '.':
            writeOnDP();
            break;
        default:
            break;
    }

}
