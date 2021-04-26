/*
 * uart_config.h
 *
 *  Created on: Mar 10, 2021
 *      Author: onemo
 */

#ifndef UART_CONFIG_H_
#define UART_CONFIG_H_


#define SYSTEM_CLOCK_FREQ       48000000
#define CS_DCO_FREQUENCY        CS_DCO_FREQUENCY_48

#define CLK_SRC                 EUSCI_A_UART_CLOCKSOURCE_SMCLK
#define UART_PRESCALE           6//78//312
#define FIRST_MOD_REG           8//2//8
#define SECOND_MOD_REG          17//0//170
#define PARITY                  EUSCI_A_UART_NO_PARITY
#define LSB_FIRST               EUSCI_A_UART_LSB_FIRST
#define NUM_STOP_BITS           EUSCI_A_UART_ONE_STOP_BIT
#define MODE                    EUSCI_A_UART_MODE
#define OVERSAMPLING            EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
#define DATA_FRAME_LENGTH       EUSCI_A_UART_8_BIT_LEN
#define UART_BASE               EUSCI_A0_BASE

#define UART_PORT               GPIO_PORT_P1
#define UART_RX_PIN             GPIO_PIN2
#define UART_TX_PIN             GPIO_PIN3


#endif /* UART_CONFIG_H_ */

/*MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
FLCTL->BANK0_RDCTL |= (FLCTL_BANK0_RDCTL_BUFI | FLCTL_BANK0_RDCTL_BUFD );
FLCTL->BANK1_RDCTL |= (FLCTL_BANK1_RDCTL_BUFI | FLCTL_BANK1_RDCTL_BUFD );
PCM_setPowerState(PCM_AM_DCDC_VCORE1);

MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

//set this to 4 (orginially 1) to see if it will work with nfc
//works, but needs to be on 2400 baud for reading, check CLI_UART
CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_4 );
*/
/* Globally enable interrupts */
//__enable_interrupt();
