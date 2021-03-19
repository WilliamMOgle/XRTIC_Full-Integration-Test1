/*
 * nfc.c
 *
 *  Created on: Mar 18, 2021
 *      Author: wills
 */

#include "nfc.h"



void NFC_emulatedT4TSniffer(void)
{
    uint16_t ui16FileID;
    uint16_t ui16StartIdx;
    uint8_t ui8BufferLength;
    uint8_t ui8T4TStatus;

    char pui8ByteBuffer[3];
    char pui8WordBuffer[5];

    ui8T4TStatus = ISO_7816_4_getCEStatus(&ui16FileID, &ui16StartIdx, &ui8BufferLength);

    if ((ui8T4TStatus == CE_READ_FLAG) && (ui8BufferLength > 1))
    {
        //Serial_printf("Read File 0x",CE_FILE_STATUS);

        // Print File ID
        //convertWordToAscii(ui16FileID,(uint8_t *)pui8WordBuffer);

        //Serial_printf(pui8WordBuffer,CE_FILE_STATUS);

        //Serial_printf(" , Idx 0x",CE_FILE_STATUS);

        // Print Index
        //convertWordToAscii(ui16StartIdx,(uint8_t *) pui8WordBuffer);

        //Serial_printf(pui8WordBuffer,CE_FILE_STATUS);

        //Serial_printf(" , Len 0x",CE_FILE_STATUS);

        // Print Len
        //convertByteToAscii(ui8BufferLength,(uint8_t *) pui8ByteBuffer);

        //Serial_printf(pui8ByteBuffer,CE_FILE_STATUS);

        pui8ByteBuffer[0] = 0x0D;
        pui8ByteBuffer[1] = 0x0A;
        pui8ByteBuffer[2] = 0x00;

        // Send New Line
        //Serial_printf(pui8ByteBuffer,CE_FILE_STATUS);
    }
    else if(ui8T4TStatus == CE_WRITE_FLAG)
    {
        //Serial_printf("Write  File 0x",CE_FILE_STATUS);

        // Print File ID
        //convertWordToAscii(ui16FileID,(uint8_t *)pui8WordBuffer);

        //Serial_printf(pui8WordBuffer,CE_FILE_STATUS);

        //Serial_printf(" , Idx 0x",CE_FILE_STATUS);

        // Print Index
        //convertWordToAscii(ui16StartIdx,(uint8_t *) pui8WordBuffer);

        //Serial_printf(pui8WordBuffer,CE_FILE_STATUS);

        //Serial_printf(" , Len 0x",CE_FILE_STATUS);

        // Print Len
        //convertByteToAscii(ui8BufferLength,(uint8_t *) pui8ByteBuffer);

        //Serial_printf(pui8ByteBuffer,CE_FILE_STATUS);

        pui8ByteBuffer[0] = 0x0D;
        pui8ByteBuffer[1] = 0x0A;
        pui8ByteBuffer[2] = 0x00;

        // Send New Line
        //Serial_printf(pui8ByteBuffer,CE_FILE_STATUS);
    }
    else
    {
        // Do nothing
    }
}

void NFC_initIDs(void)
{
    // NFC ID's
    uint8_t pui8NfcAId[10] = {0x08,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};   // Generic ISO14443 T4TA Tag
    uint8_t pui8NfcBId[4] = {0x08,0x0A, 0xBE,0xEF}; // Generic ISO14443 T4TB Tag
    uint8_t pui8NfcFId[8] = {0x01,0xFE,0x88,0x77,0x66,0x55,0x44,0x33};  // Type F ID for P2P

    // Set the NFC Id's for Type A, Type B, and Type F
    NFC_A_setNfcAId(pui8NfcAId,4);
    NFC_B_setNfcBId(pui8NfcBId,4);
    NFC_F_setNfcId2(pui8NfcFId,8);
}

void NFC_debugPinInit()
{
    // Initialize Debug Pins as output
    NFC_RF_FIELD_LED_DIR |= NFC_RF_FIELD_LED_BIT;
    NFC_HOST_LED_DIR |= NFC_HOST_LED_BIT;

    NFC_RW_LED_PDIR |= NFC_RW_LED_BIT;
    NFC_P2P_LED_PDIR |= NFC_P2P_LED_BIT;
    NFC_CE_LED_PDIR |= NFC_CE_LED_BIT;

    // Clear NFC pins
    NFC_RF_FIELD_LED_POUT &= ~NFC_RF_FIELD_LED_BIT;
    NFC_HOST_LED_POUT &= ~NFC_HOST_LED_BIT;

    NFC_RW_LED_POUT &= ~NFC_RW_LED_BIT;
    NFC_P2P_LED_POUT &= ~NFC_P2P_LED_BIT;
    NFC_CE_LED_POUT &= ~NFC_CE_LED_BIT;
}

void NFC_completeInit()
{
    //Initialize LP LEDs for NFC
    NFC_debugPinInit();

    //Enable interrupts globally
    __enable_interrupt();

    // Initialize USB Communication
    //Serial_init();

    // Initialize TRF7970
    TRF79x0_init();

    //Buttons_init(BUTTON_ALL);
    //Buttons_interruptEnable(BUTTON_ALL);

    TRF79x0_idleMode();

    // Initialize the NFC Controller
    NFC_init();

    // This function will configure all the settings for each protocol
    NFC_configuration();

    // Initialize Type 4 Tag RTD Message
    T4T_CE_initNDEF();

    // Initialize IDs for NFC-A, NFC-B and NFC-F
    NFC_initIDs();

#if NFC_READER_WRITER_ENABLED
    // Initialize the RW T2T, T3T, T4T and T5 state machines
    T2T_init(g_ui8TxBuffer,256);
    T3T_init(g_ui8TxBuffer,256);
    T4T_init(g_ui8TxBuffer,256);
    T5T_init(g_ui8TxBuffer,256);
#endif

}
