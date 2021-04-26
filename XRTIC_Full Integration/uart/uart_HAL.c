/*
 * uart_HAL.c
 *
 *  Created on: Mar 10, 2021
 *      Author: onemo
 */



#include "uart_HAL.h"


void initUART()
{

    eUSCI_UART_ConfigV1 uartConfig;

    //Set up Uart for 9600 baud rate
    uartConfig.selectClockSource = CLK_SRC;
    uartConfig.clockPrescalar = UART_PRESCALE;

    uartConfig.firstModReg = FIRST_MOD_REG;
    uartConfig.secondModReg = SECOND_MOD_REG;
    uartConfig.parity = PARITY;
    uartConfig.msborLsbFirst = LSB_FIRST;
    uartConfig.numberofStopBits = NUM_STOP_BITS;
    uartConfig.uartMode = MODE;
    uartConfig.overSampling = OVERSAMPLING;
    uartConfig.dataLength = DATA_FRAME_LENGTH;

    GPIO_setAsPeripheralModuleFunctionOutputPin(UART_PORT, UART_RX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);     //RX
    GPIO_setAsPeripheralModuleFunctionOutputPin(UART_PORT, UART_TX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);     //TX

    //enable and initialize UART
    UART_initModule(UART_BASE , &uartConfig);
    UART_enableModule(UART_BASE);


}

void initMCU()
{
    //Need for when clock is set to 48mHz
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);
    FLCTL->BANK0_RDCTL |= (FLCTL_BANK0_RDCTL_BUFI | FLCTL_BANK0_RDCTL_BUFD );
    FLCTL->BANK1_RDCTL |= (FLCTL_BANK1_RDCTL_BUFI | FLCTL_BANK1_RDCTL_BUFD );
    PCM_setPowerState(PCM_AM_DCDC_VCORE1);

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_4 );
}

void transmitChar(uint8_t c)
{
    UART_transmitData(UART_BASE, c);
}

void transmitNewLine()
{
    UART_transmitData(UART_BASE, '\n');
    UART_transmitData(UART_BASE, '\r');
}

void transmitString(uint8_t* s)
{
    uint8_t x = 0;
    while(s[x] != 0)
    {
        UART_transmitData(UART_BASE, s[x]);
        x++;
    }

}

void transmitInt(int i)
{
    char tempC;
    int tempI;
    int digit;
    int place = 10;

    if(i < 0)
    {
        transmitChar('-');
        tempI = (-1) * i;
    }
    else
    {
        tempI = i;
    }

    while(tempI / place != 0)
    {
        place *= 10;
    }
    while(place > 1)
    {
        place /= 10;
        digit = tempI / place;
        tempI = tempI % place;
        tempC = digit + '0';
        transmitChar(tempC);
    }

}

void transmitIntHex(int i)
{

}
