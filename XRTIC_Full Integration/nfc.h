/*
 * nfc.h
 *
 *  Created on: Mar 18, 2021
 *      Author: wills
 */

#ifndef NFC_H_
#define NFC_H_

#include "nfc_controller.h"
#include <tag_header.h>

//
// Buffer to store incoming packets from NFC host
//
uint8_t g_ui8SerialBuffer[265];




bool g_bEnableAutoSDD;
bool g_bExtAmplifier;
bool g_bTRF5VSupply;
tTRF79x0_Version g_eTRFVersion;
bool g_bSupportCertification;
uint16_t g_ui16ListenTime;

//////////////////////////////////////////////////
tT5TStateMachine g_eT5TState;

static uint16_t g_ui16T5TBlockNumber;
static uint16_t g_ui16T5TBlockCount;
static uint8_t g_pui8T5TRxBuffer[30];

bool g_bT5TWaitForRsp;

uint8_t * g_pui8T5TBuffer;

uint16_t g_ui16T5TNdefLen;

uint16_t g_ui16T5TMaxNdefLen;

uint16_t g_ui16T5TNdefIndex;

uint16_t g_ui16T5TTLVRemaining;

uint8_t g_ui8T5TCurrentTlv;

uint16_t g_ui16T5TSize;

bool g_bT5TTLVSelected;

bool g_bT5TTLVLengthKnown;

uint8_t g_ui8T5TTLVLengthRemainBytes;

uint16_t g_ui16T5TReadIndex;

bool g_bT5TFormatting;

uint8_t g_ui8TxBuffer[256];
uint8_t g_ui8TxLength;


#if NFC_READER_WRITER_ENABLED
    t_sNfcRWMode g_sRWSupportedModes;
    t_sNfcRWCommBitrate g_sRWSupportedBitrates;
    t_sIsoDEP_RWSetup g_sRWSetupOptions;
    uint8_t g_ui8IsoDepInitiatorDID;
#endif

void NFC_configuration(void);
void Serial_processCommand(void);
void LCD_init(void);
void updateLcdfcStatus(bool bUpdateRssiOnly);
void NFC_emulatedT4TSniffer(void);
void NFC_initIDs(void);
void NFC_debugPinInit();
void NFC_completeInit();        //contains many NFC init functions


#endif /* NFC_H_ */
