//*****************************************************************************
//
// main.c
//
// Copyright (c) 2015 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************
#include <driverlib.h>
#include "nfc_controller.h"

/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>

void printString(char output[]);

#ifdef MSP432P401R_LAUNCHPAD_ENABLED
#include "lp_buttons.h"
#endif

#define DOGS102x6_DRAW_NORMAL 0x00
#define DOGS102x6_DRAW_INVERT 0x01


#include <tag_header.h>


#include "bump_sensor.h"
#include "jsonParser.h"

//ROVER INCLUDES AND MQTT INCLUDES
// Standard includes
#include <stdlib.h>
#include <string.h>

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "simplelink.h"
#include "sl_common.h"
#include "MQTTClient.h"


//UART INCLUDES
#include "uart_HAL.h"

//ROVER INCLUDES
//#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
//#include <stdint.h>
//#include <stdbool.h>
#include "RSLK_Wheel.h"
#include "Timer_HAL.h"
#include "RSLK_Rover.h"
#include <pwm_HAL.h>

//ROVER DEFINES
#define SYS_CLK 48000000



//ROVER AND MQTT STUFF

/*
 * Values for below macros shall be modified per the access-point's (AP) properties
 * SimpleLink device will connect to following AP when the application is executed
 */
/*
 * Matt's Information
 */

//#define SSID_NAME       "DuhFastStuff2.4"       // Access point name to connect to.
//#define PASSKEY         "bday0628"              //Password in case of secure AP

/*
 * Amar's Information
 */

#define SSID_NAME       "VeeCastFox178"       // Access point name to connect to.
#define PASSKEY         "VeeLearCar256!%"              //Password in case of secure AP

#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2     /* Security type of the Access piont */
#define PASSKEY_LEN     pal_Strlen(PASSKEY)      /* Password length in case of secure AP */

/*
 * MQTT server and topic properties that shall be modified per application
 */
#define MQTT_BROKER_SERVER  "s1.airmqtt.com"
#define MQTT_BROKER_PORT 10004
#define MQTT_BROKER_USERNAME "a77buelx"
#define MQTT_BROKER_PASSWORD "4pixgqnd"

#define SUBSCRIBE_TOPIC "XRTIC20/Commands/Rover"
#define PUBLISH_TOPIC "/msp/cc3100/fromLP"

// MQTT message buffer size
#define BUFF_SIZE 32


#define APPLICATION_VERSION "1.0.0"

#define MCLK_FREQUENCY 48000000
#define PWM_PERIOD 255

#define SL_STOP_TIMEOUT        0xFF

#define SMALL_BUF           32
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   1024

/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

#define min(X,Y) ((X) < (Y) ? (X) : (Y))


/*
 * GLOBAL VARIABLES -- Start
 */
/* Button debounce state variables */
volatile unsigned int S1buttonDebounce = 0;
volatile unsigned int S2buttonDebounce = 0;
volatile int publishID = 0;

unsigned char macAddressVal[SL_MAC_ADDR_LEN];
unsigned char macAddressLen = SL_MAC_ADDR_LEN;

char macStr[18];        // Formatted MAC Address String
char uniqueID[9];       // Unique ID generated from TLV RAND NUM and MAC Address

Network n;
Client hMQTTClient;     // MQTT Client

_u32  g_Status = 0;
struct{
    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
    _u8 SendBuff[MAX_SEND_BUF_SIZE];

    _u8 HostName[SMALL_BUF];
    _u8 CityName[SMALL_BUF];

    _u32 DestinationIP;
    _i16 SockID;
}g_AppData;

/* Port mapper configuration register */

//WILLI USING A03, A04
/*
const uint8_t port_mapping[] =
{
    //Port P2:
    //PM_TA0CCR1A, PM_TA0CCR2A, PM_TA0CCR3A, PM_NONE, PM_TA1CCR1A, PM_NONE, PM_NONE, PM_NONE
    PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_TA1CCR1A, PM_NONE, PM_NONE, PM_NONE
};
*/

/* TimerA UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfigMQTT =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_8,          // SMCLK/8 = 6MHz
        90000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

/*
 * GLOBAL VARIABLES -- End
 */


/*
 * STATIC FUNCTION DEFINITIONS -- Start
 */
static _i32 establishConnectionWithAP();
static _i32 configureSimpleLinkToDefaultState();
static _i32 initializeAppVariables();
static void displayBanner();
static void messageArrived(MessageData*);
static void generateUniqueID();

//END ROVER AND MQTT STUFF














//
// Buffer to store incoming packets from NFC host
//
uint8_t g_ui8SerialBuffer[265];

//
// Number of bytes received from the host
//
volatile uint16_t g_ui16BytesReceived = 0x00;

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

//////////////////////////////////////////////////

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

void turnOn_LaunchpadLED1();
void turnOff_LaunchpadLED1();
void initialize_LaunchpadLED1();
void toggle_LaunchpadLED1();

void initialize_LaunchpadLED2_green();
void turnOn_LaunchpadLED2_green();
void turnOff_LaunchpadLED2_green();
void toggle_LaunchpadLED2_green();

void initialize_LaunchpadLED2_blue();
void turnOn_LaunchpadLED2_blue();
void turnOff_LaunchpadLED2_blue();
void toggle_LaunchpadLED2_blue();

uint8_t g_ui8TxBuffer[256];
uint8_t g_ui8TxLength;

void LCD_stringDraw(uint8_t row, uint8_t col, char *word, uint8_t style)
{

}

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

int main(int argc, char** argv)
{






    bool tagReseted = true;

    tNfcState eTempNFCState;
    tNfcState eCurrentNFCState;

    // CE Variables
    t_sNfcCEMode sCEMode;

    t_sNfcP2PMode sP2PMode;
    t_sNfcP2PCommBitrate sP2PBitrate;

    // Reader/Writer RX Status
    t_sNfcRWMode sRWMode;
    t_sNfcRWCommBitrate sRWBitrate;

    // Initialize MCU
    MCU_init();

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



    //START SETUP FOR ROVER AND MQTT
    MAP_WDT_A_holdTimer();

    //ROVER addition
    //rslk rover init
    initRSLKRover(SYS_CLK);
    initRSLKTimer32(RSLK_TIMER32_BASE);
    enableWheel(&right_wheel_data);
    setWheelDirFwrd(&right_wheel_data);
    enableWheel(&left_wheel_data);
    setWheelDirFwrd(&left_wheel_data);




    setWheelDutyCycle(&right_wheel_data, 0.1);
    wheelUpdateMove(&right_wheel_data);

    setWheelDutyCycle(&left_wheel_data, 0.1);
    wheelUpdateMove(&left_wheel_data);


    initMCU();
    initUART();

    transmitString("Hey");

    _i32 retVal = -1;

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    // Stop WDT and initialize the system-clock of the MCU
    stopWDT();
    initClk();

    // GPIO Setup for Pins 2.0-2.2
    //MAP_PMAP_configurePorts((const uint8_t *) port_mapping, PMAP_P2MAP, 1,
    //    PMAP_DISABLE_RECONFIGURATION);

    //MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
    //    GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    //Confinguring P1.1 & P1.4 as an input and enabling interrupts
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    //STUFF FROM HERE TO END NEEDS TO STAY???

    /* Configure TimerA0 for RGB LED*/
    //TA0CCR0 = PWM_PERIOD;                   // PWM Period
    //TA0CCTL1 = OUTMOD_7;                    // CCR1 reset/set
    //TA0CCR1 = PWM_PERIOD * (0/255);                 // CCR1 PWM duty cycle
    //TA0CCTL2 = OUTMOD_7;                    // CCR2 reset/set
    //TA0CCR2 = PWM_PERIOD * (0/255);                 // CCR2 PWM duty cycle
    //TA0CCTL3 = OUTMOD_7;                    // CCR3 reset/set
    //TA0CCR3 = PWM_PERIOD * (0/255);                 // CCR3 PWM duty cycle
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;  // SMCLK, up mode, clear TAR

    //Configuring TimerA1 for Up Mode
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfigMQTT);

    Interrupt_enableInterrupt(INT_TA1_0);
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableMaster();

    //END

    /* Configure command line interface */
    //CLI_Configure();

    displayBanner();


    /*
     * Following function configures the device to default state by cleaning
     * the persistent settings stored in NVMEM (viz. connection profiles &
     * policies, power policy etc)
     *
     * Applications may choose to skip this step if the developer is sure
     * that the device is in its default state at start of application
     *
     * Note that all profiles and persistent settings that were done on the
     * device will be lost
     */


    retVal = configureSimpleLinkToDefaultState();
    if(retVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == retVal)
            transmitString(" Failed to configure the device in its default state \n\r");

        LOOP_FOREVER();
    }

    transmitString(" Device is configured in default state \n\r");

    /*
     * Assumption is that the device is configured in station mode already
     * and it is in its default state
     */


    retVal = sl_Start(0, 0, 0);
    if ((retVal < 0) ||
        (ROLE_STA != retVal) )
    {
        transmitString(" Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    transmitString(" Device started as STATION \n\r");

    // Connecting to WLAN AP
    retVal = establishConnectionWithAP();
    if(retVal < 0)
    {
        transmitString(" Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }

    transmitString(" Connection established w/ AP and IP is acquired \n\r");

    // Obtain MAC Address
    sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);

    // Print MAC Addres to be formatted string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            macAddressVal[0], macAddressVal[1], macAddressVal[2], macAddressVal[3], macAddressVal[4], macAddressVal[5]);

    // Generate 32bit unique ID from TLV Random Number and MAC Address
    generateUniqueID();

    int rc = 0;
    unsigned char buf[100];
    unsigned char readbuf[100];

    NewNetwork(&n);
    rc = ConnectNetwork(&n, MQTT_BROKER_SERVER, MQTT_BROKER_PORT);

    if (rc != 0) {
        transmitString(" Failed to connect to MQTT broker \n\r");
        LOOP_FOREVER();
    }
    transmitString(" Connected to MQTT broker \n\r");

    MQTTClient(&hMQTTClient, &n, 1000, buf, 100, readbuf, 100);
    MQTTPacket_connectData cdata = MQTTPacket_connectData_initializer;
    cdata.MQTTVersion = 3;
    cdata.clientID.cstring = uniqueID;
    cdata.username.cstring = MQTT_BROKER_USERNAME;
    cdata.password.cstring = MQTT_BROKER_PASSWORD;
    rc = MQTTConnect(&hMQTTClient, &cdata);

    if (rc != 0) {
        transmitString(" Failed to start MQTT client \n\r");
        LOOP_FOREVER();
    }
    transmitString(" Started MQTT client successfully \n\r");

    rc = MQTTSubscribe(&hMQTTClient, SUBSCRIBE_TOPIC, QOS0, messageArrived);

    if (rc != 0) {
        transmitString(" Failed to subscribe to /msp/cc3100/demo topic \n\r");
        LOOP_FOREVER();
    }
    transmitString(" Subscribed to /msp/cc3100/demo topic \n\r");

    rc = MQTTSubscribe(&hMQTTClient, uniqueID, QOS0, messageArrived);

    if (rc != 0) {
        transmitString(" Failed to subscribe to uniqueID topic \n\r");
        LOOP_FOREVER();
    }
    transmitString(" Subscribed to uniqueID topic \n\r");




    //ROVER MOVE FUNCTION
    //moveForwardForTime(270, 2000);      //move forward at 27 RPM for 2 seconds






    while(1)
    {
        eTempNFCState = NFC_run();

        if(eTempNFCState == NFC_DATA_EXCHANGE_PROTOCOL)
        {
            if(NFC_RW_getModeStatus(&sRWMode,&sRWBitrate))
            {
#if NFC_READER_WRITER_ENABLED
                NFC_RW_LED_POUT |= NFC_RW_LED_BIT;

                if( sRWMode.bits.bNfcA == 1)
                {
                    if(NFC_A_getSAK() == 0x00)
                    {
                        // T2T Tag State Machine
                        if (!tagReseted){
                            tagReseted = true;
                            toggle_LaunchpadLED2_green();
                        }
                        T2T_stateMachine();
                    }
                    else if(NFC_A_getSAK() & 0x20)
                    {
                        // T4T Tag State Machine
                        T4T_stateMachine();
                    }
                }
                else if(sRWMode.bits.bNfcB == 1)
                {
                    if(NFC_B_isISOCompliant())
                    {
                        // T4T Tag State Machine
                        T4T_stateMachine();
                    }
                }
                else if(sRWMode.bits.bNfcF == 1)
                {
                    // T3T Tag State Machine
                    T3T_stateMachine();
                }
                else if(sRWMode.bits.bISO15693 == 1)
                {
                    // T5T Tag State Machine
                    if (!tagReseted){
                        tagReseted = true;
                        toggle_LaunchpadLED1();
                    }
                    T5T_stateMachine();
                }
#endif
            }
            else if(NFC_P2P_getModeStatus(&sP2PMode,&sP2PBitrate))
            {

            }
            else if(NFC_CE_getModeStatus(&sCEMode))
            {

            }

            // Update only RSSI
            updateLcdfcStatus(true);
        }
        else
        {
            // Clear LEDs (RX & TX)
            turnOff_LaunchpadLED1();
            turnOff_LaunchpadLED2_red();//LaunchpadLED2_green
            turnOff_LaunchpadLED2_green();
            turnOff_LaunchpadLED2_blue();
            tagReseted = false;
        }

        // Update Current State if it has changed.
        if(eCurrentNFCState != eTempNFCState)
        {
            __no_operation();

            if(eCurrentNFCState != NFC_TARGET_WAIT_FOR_ACTIVATION
                && eCurrentNFCState != NFC_STATE_IDLE
                && (eTempNFCState == NFC_PROTOCOL_ACTIVATION
                    || eTempNFCState == NFC_DISABLED))
            {
                eCurrentNFCState = eTempNFCState;

#if NFC_READER_WRITER_ENABLED
                // Initialize the RW T2T, T3T, T4T and T5 state machines
                T2T_init(g_ui8TxBuffer,256);
                T3T_init(g_ui8TxBuffer,256);
                T4T_init(g_ui8TxBuffer,256);
                T5T_init(g_ui8TxBuffer,256);
#endif

                // Clear RW, P2P and CE LEDs
                NFC_RW_LED_POUT &= ~NFC_RW_LED_BIT;
                NFC_P2P_LED_POUT &= ~NFC_P2P_LED_BIT;
                NFC_CE_LED_POUT &= ~NFC_CE_LED_BIT;

                buttonDebounce = 1;

                //Serial_printf("DC",NFC_MODE_LOST);
            }
            else
            {
                eCurrentNFCState = eTempNFCState;
            }

        }

        // Check if any packets have been received from the NFC host.
        if(g_ui16BytesReceived > 0)
        {
            Serial_processCommand();
        }




        rc = MQTTYield(&hMQTTClient, 10);
        if (rc != 0) {
            transmitString(" MQTT failed to yield \n\r");
            LOOP_FOREVER();
        }

        if (publishID) {
            int rc = 0;
            MQTTMessage msg;
            msg.dup = 0;
            msg.id = 0;
            msg.payload = uniqueID;
            msg.payloadlen = 8;
            msg.qos = QOS0;
            msg.retained = 0;
            rc = MQTTPublish(&hMQTTClient, PUBLISH_TOPIC, &msg);

            if (rc != 0) {
                transmitString(" Failed to publish unique ID to MQTT broker \n\r");
                LOOP_FOREVER();
            }
            transmitString(" Published unique ID successfully \n\r");

            publishID = 0;
        }




        //BUMP SENSOR CHECKS
        if(bumpSensorPressed(BUMP0))
            transmitString("Bump 0 Pressed!\n\r");
        if(bumpSensorPressed(BUMP1))
            transmitString("Bump 1 Pressed!\n\r");
        if(bumpSensorPressed(BUMP2))
            transmitString("Bump 2 Pressed!\n\r");
        if(bumpSensorPressed(BUMP3))
            transmitString("Bump 3 Pressed!\n\r");
        if(bumpSensorPressed(BUMP4))
            transmitString("Bump 4 Pressed!\n\r");
        if(bumpSensorPressed(BUMP5))
            transmitString("Bump 5 Pressed!\n\r");


    }

}


void T5T_init(uint8_t * pui8Ndef, uint16_t ui16NdefMaxSize)
{
    g_eT5TState = T5T_INVENTORY;
    g_ui16T5TBlockNumber = 0;
    g_ui16T5TBlockCount = 0;
    g_bT5TWaitForRsp = false;

    g_pui8T5TBuffer = pui8Ndef;

    g_ui16T5TMaxNdefLen = ui16NdefMaxSize;

    g_ui16T5TNdefIndex = 0;

    g_ui16T5TNdefLen = 0;

    g_ui16T5TTLVRemaining = 0;

    g_ui8T5TCurrentTlv = 0;

    g_ui16T5TSize = 0;

    g_bT5TTLVSelected = false;

    g_bT5TTLVLengthKnown = false;

    g_ui8T5TTLVLengthRemainBytes = 0;

    g_ui16T5TReadIndex = 0;

    g_bT5TFormatting = false;
}

void T5T_writeNDEF(uint8_t * pui8Data, uint16_t ui16NdefLen)
{
    uint8_t ui8Offset;

    ui8Offset = 0;

    if(g_eT5TState == T5T_SELECTED_IDLE)
    {
        // Total Size of tag - 3 bytes (of TLV overhead Type, Length and Terminator TLV) - 4 bytes (Capability Container)
        if(g_ui16T5TSize > 7 && (g_ui16T5TSize-7) >= ui16NdefLen)
        {
            g_eT5TState = T5T_WRITE_NDEF;

            g_ui16T5TNdefIndex = 0;

            // Start Writing at Block 1
            g_ui16T5TBlockNumber = 0x01;

            // NDEF Msg TLV - Type
            g_pui8T5TBuffer[ui8Offset++] = TLV_NDEF_MSG;
            // NDEF Msg TLV - Length
            if(ui16NdefLen < 0xFF)
            {
                g_pui8T5TBuffer[ui8Offset++] = (uint8_t) ui16NdefLen;
            }

            g_ui16T5TTLVRemaining = (uint16_t) ui8Offset+ui16NdefLen+1;

            memcpy(&g_pui8T5TBuffer[ui8Offset],pui8Data,ui16NdefLen);

            g_pui8T5TBuffer[ui8Offset+ui16NdefLen] = TLV_TERMINATOR;
        }
        else
        {
            //Serial_printf("T5T Write Fail: NDEF message size not supported by tag.\n",RW_STATUS_DATA);
        }

    }
    else
    {
        //Serial_printf("T5T Write Fail: Busy.\n",RW_STATUS_DATA);
    }
}

void T5T_formatTag(void)
{
    uint8_t ui8TagBlockSize;

    //Serial_printf("T5T Formatting...\n",RW_STATUS_DATA);

    ui8TagBlockSize = NFC_RW_T5T_getVICCBlockSize();

    if(g_eT5TState == T5T_SELECTED_IDLE)
    {
        g_eT5TState = T5T_WRITE_NDEF;

        g_bT5TFormatting = true;

        g_ui16T5TNdefIndex = 0;

        g_ui16T5TBlockNumber = 0;

        // Check for Extended Memory Tags
        if (g_ui16T5TBlockCount > 255)
        {
            g_ui16T5TTLVRemaining = 12;         // Need 2 Write Blocks to finish writing CC

            g_pui8T5TBuffer[0] = 0xE2;          // Tags with more than 256 blocks have Magic Number = 0xE2
            g_pui8T5TBuffer[1] = 0x40;          // NFC Major Version = 1, NFC Minor Version = 0, Read Access = Always, Write Access = Always
            g_pui8T5TBuffer[2] = 0x00;

            if (g_ui16T5TBlockCount == 0x0A)
            {
                g_pui8T5TBuffer[3] = 0x10;      // TI Tag-It Pro/Standard needs Option Flag
            }
            else
            {
                g_pui8T5TBuffer[3] = 0x00;
            }

            g_pui8T5TBuffer[4] = 0x00;          // RFU
            g_pui8T5TBuffer[5] = 0x00;          // RFU
            g_pui8T5TBuffer[6] = (uint8_t) ((g_ui16T5TBlockCount & 0xFF00) >> 8);
            g_pui8T5TBuffer[7] = (uint8_t) (g_ui16T5TBlockCount && 0x00FF);

            // Write an empty NDEF to tag
            g_pui8T5TBuffer[8] = 0x03;
            g_pui8T5TBuffer[9] = 0x00;
            g_pui8T5TBuffer[10] = 0xFE;
            g_pui8T5TBuffer[11] = 0x00;
        }
        else
        {
            g_ui16T5TTLVRemaining = 8;          // Need 1 Write Block for CC

            g_pui8T5TBuffer[0] = 0xE1;          // All other tags have Magic Number = 0xE1
            g_pui8T5TBuffer[1] = 0x40;          // NFC Major Version = 1, NFC Minor Version = 0, Read Access = Always, Write Access = Always

            if (g_ui16T5TBlockCount == 0x0A)
            {
                g_pui8T5TBuffer[2] = 0x04;
                g_pui8T5TBuffer[3] = 0x10;      // TI Tag-It Pro/Standard needs Option Flag
            }
            else
            {
                g_pui8T5TBuffer[2] = (uint8_t) (((g_ui16T5TBlockCount+1)*(ui8TagBlockSize+1))>>3);
                g_pui8T5TBuffer[3] = 0x00;
            }

            // Write an empty NDEF to tag
            g_pui8T5TBuffer[4] = 0x03;
            g_pui8T5TBuffer[5] = 0x00;
            g_pui8T5TBuffer[6] = 0xFE;
            g_pui8T5TBuffer[7] = 0x00;
        }
    }
    else
    {
        //Serial_printf("T5T Write Fail: Busy./n",RW_STATUS_DATA);
    }
}

void T5T_stateMachine(void)
{
    uint8_t ui8RxLength;
    uint8_t * pui8RxData;
    tNfcRwT5TConnectionStatus eT5TStatus;
    char pui8LenBuffer[5];
    uint8_t ui8Temp;
    uint8_t ui8LocalReadIndex;

    // Waiting for a response
    if(g_bT5TWaitForRsp == true)
    {
        switch(g_eT5TState)
        {
        case T5T_INVENTORY:
            eT5TStatus = NFC_RW_T5T_getInventoryStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_INVENTORY_SUCCESS)
            {
                g_eT5TState = T5T_GET_SYS_INFO_EXT;
            }
            else
            {
                g_eT5TState = T5T_SELECTED_IDLE;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_GET_SYS_INFO_EXT:
            eT5TStatus = NFC_RW_T5T_getGetSysInfoStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_SUCCESS)
            {
                NFC_RW_T5T_getPacketStatus(&pui8RxData,&ui8RxLength);

                memcpy(g_pui8T5TRxBuffer,pui8RxData,ui8RxLength);

                g_eT5TState = T5T_READ_CC;

                g_ui16T5TBlockCount = NFC_RW_T5T_getVICCBlockCount();
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_FAIL)
            {
                g_eT5TState = T5T_GET_SYS_INFO_1;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_GET_SYS_INFO_1:
            eT5TStatus = NFC_RW_T5T_getGetSysInfoStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_SUCCESS)
            {
                NFC_RW_T5T_getPacketStatus(&pui8RxData,&ui8RxLength);

                memcpy(g_pui8T5TRxBuffer,pui8RxData,ui8RxLength);

                g_eT5TState = T5T_READ_CC;

                g_ui16T5TBlockCount = NFC_RW_T5T_getVICCBlockCount();
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_FAIL)
            {
                g_eT5TState = T5T_GET_SYS_INFO_2;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_GET_SYS_INFO_2:
            eT5TStatus = NFC_RW_T5T_getGetSysInfoStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_SUCCESS)
            {
                g_eT5TState = T5T_READ_CC;

                g_ui16T5TBlockCount = NFC_RW_T5T_getVICCBlockCount();
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_FAIL)
            {
                g_eT5TState = T5T_READ_CC;

                g_ui16T5TBlockCount = 0x0A;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_READ_CC:
            eT5TStatus = NFC_RW_T5T_getReadSingleStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_SUCCESS)
            {
                NFC_RW_T5T_getPacketStatus(&pui8RxData,&ui8RxLength);

                memcpy(g_pui8T5TRxBuffer,pui8RxData+1,ui8RxLength-1);

                // Check if NDEF message formatted
                if(g_pui8T5TRxBuffer[0] == 0xE1)
                {
                    // Switch to the read NDEF data
                    g_eT5TState = T5T_READ_NDEF;

                    g_ui16T5TSize = g_pui8T5TRxBuffer[2] << 3;
                }
                else
                {
                    // Switch to the read NDEF data
                    g_eT5TState = T5T_READ_DATA;

                    //Serial_printf("T5T Not NDEF Formatted",RW_STATUS_DATA);

                    // Print New Line
                    //Serial_printf("\n",RW_STATUS_DATA);

                    //Serial_printf("Block ",RW_STATUS_DATA);

                    // Print Block Number
                    //convertWordToAscii(g_ui16T5TBlockNumber,(uint8_t *) pui8LenBuffer);

                    //Serial_printf(pui8LenBuffer,RW_STATUS_DATA);

                    //Serial_printf(" ",RW_STATUS_DATA);

                    for(ui8Temp = 0; ui8Temp < (ui8RxLength-1); ui8Temp++)
                    {
                        //convertByteToAscii(g_pui8T5TRxBuffer[ui8Temp],(uint8_t *) pui8LenBuffer);

                        //Serial_printf("0x",RW_STATUS_DATA);

                        //Serial_printf(pui8LenBuffer,RW_STATUS_DATA);

                        //Serial_printf(" ",RW_STATUS_DATA);
                    }

                    // Print New Line
                    //Serial_printf("\n",RW_STATUS_DATA);

                    g_ui16T5TSize = 0;
                }

                // Incrment Block Number
                g_ui16T5TBlockNumber++;
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_FAIL)
            {
                g_eT5TState = T5T_SELECTED_IDLE;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_READ_DATA:
            eT5TStatus = NFC_RW_T5T_getReadSingleStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_SUCCESS)
            {
                NFC_RW_T5T_getPacketStatus(&pui8RxData,&ui8RxLength);

                memcpy(g_pui8T5TRxBuffer,pui8RxData+1,ui8RxLength-1);

                //Serial_printf("Block ",RW_STATUS_DATA);

                // Print Block Number
                //convertWordToAscii(g_ui16T5TBlockNumber,(uint8_t *) pui8LenBuffer);

                //Serial_printf(pui8LenBuffer,RW_STATUS_DATA);

                //Serial_printf(" ",RW_STATUS_DATA);

                for(ui8Temp = 0; ui8Temp < (ui8RxLength-1); ui8Temp++)
                {
                    //convertByteToAscii(g_pui8T5TRxBuffer[ui8Temp],(uint8_t *) pui8LenBuffer);

                    //Serial_printf("0x",RW_STATUS_DATA);

                    //Serial_printf(pui8LenBuffer,RW_STATUS_DATA);

                    //Serial_printf(" ",RW_STATUS_DATA);
                }

                // Print New Line
                //Serial_printf("\n",RW_STATUS_DATA);

                g_ui16T5TBlockNumber++;

                if(g_ui16T5TBlockNumber > g_ui16T5TBlockCount)
                {
                    g_eT5TState = T5T_SELECTED_IDLE;
                }
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_FAIL)
            {
                g_eT5TState = T5T_SELECTED_IDLE;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_READ_NDEF:
            eT5TStatus = NFC_RW_T5T_getReadSingleStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_SUCCESS)
            {
                NFC_RW_T5T_getPacketStatus(&pui8RxData,&ui8RxLength);

                memcpy(g_pui8T5TRxBuffer,pui8RxData+1,ui8RxLength-1);

                ui8RxLength = ui8RxLength - 1;

                ui8LocalReadIndex = 0x00;

                // Processes all the bytes returned from read block #
                while(ui8LocalReadIndex < ui8RxLength && g_eT5TState == T5T_READ_NDEF)
                {

                    // TLV handling

                    //  No TLV is selected
                    if(g_bT5TTLVSelected == false)
                    {
                        // Read TLV
                        g_ui8T5TCurrentTlv =  g_pui8T5TRxBuffer[ui8LocalReadIndex++];

                        // NULL TLV just increment the read index
                        if(g_ui8T5TCurrentTlv == TLV_NULL)
                        {
                            // Do nothing
                        }
                        // TERMINATOR TLV we are done reading
                        else if(g_ui8T5TCurrentTlv == TLV_TERMINATOR)
                        {
                            g_eT5TState = T5T_SELECTED_IDLE;
                        }
                        else
                        {
                            // Go read length
                            g_bT5TTLVSelected = true;

                            g_bT5TTLVLengthKnown = false;
                            // Default
                            g_ui8T5TTLVLengthRemainBytes = 0xFF;
                        }

                    }
                    // TLV is selected and Length not known
                    else if(g_bT5TTLVLengthKnown == false)
                    {
                        // No length bytes have been read - (0xFF is default value)
                        if(g_ui8T5TTLVLengthRemainBytes == 0xFF)
                        {
                            // Large NDEF - NDEF length is stored in two subsequent bytes
                            if(g_pui8T5TRxBuffer[ui8LocalReadIndex] == 0xFF)
                            {
                                g_ui8T5TTLVLengthRemainBytes = 2;
                            }
                            else
                            {
                                g_bT5TTLVLengthKnown = true;
                                g_ui16T5TNdefLen = g_pui8T5TRxBuffer[ui8LocalReadIndex];
                                g_ui16T5TTLVRemaining = g_ui16T5TNdefLen;
                                g_ui16T5TReadIndex = 0;
                            }
                        }
                        // Read 1st length byte
                        else if(g_ui8T5TTLVLengthRemainBytes == 2)
                        {
                            g_ui16T5TNdefLen = (uint16_t) (g_pui8T5TRxBuffer[ui8LocalReadIndex] << 8);
                            g_ui8T5TTLVLengthRemainBytes = 1;
                        }
                        else if(g_ui8T5TTLVLengthRemainBytes == 1)
                        {
                            g_bT5TTLVLengthKnown = true;
                            g_ui16T5TNdefLen |= ((uint16_t) g_pui8T5TRxBuffer[ui8LocalReadIndex] & 0xFF);
                            g_ui16T5TTLVRemaining = g_ui16T5TNdefLen;
                            g_ui8T5TTLVLengthRemainBytes = 0;
                            g_ui16T5TReadIndex = 0;
                        }
                        else
                        {
                            // Do nothing
                        }
                        // Increment read index
                        ui8LocalReadIndex++;
                    }
                    // Parse the selected TLV data
                    else
                    {
                        if(g_ui8T5TCurrentTlv == TLV_NDEF_MSG)
                        {
                            // If the current TLV remaining bytes are more than the available bytes
                            if(g_ui16T5TTLVRemaining > (ui8RxLength - ui8LocalReadIndex))
                            {
//                              //Serial_printBuffer((char *)&g_pui8T5TRxBuffer[ui8ReadIndex],(15 - ui8ReadIndex),RW_PAYLOAD_DATA);

                                // Store Incoming NDEF Message into g_pui8T5TBuffer
                                if(g_ui16T5TMaxNdefLen > (g_ui16T5TReadIndex + (ui8RxLength - ui8LocalReadIndex)))
                                {
                                    memcpy(&g_pui8T5TBuffer[g_ui16T5TReadIndex],&g_pui8T5TRxBuffer[ui8LocalReadIndex],(uint16_t) ui8RxLength - ui8LocalReadIndex);
                                }
                                else
                                {
                                    // NDEF message is larger than the size of the g_pui8T5TBuffer buffer
                                }

                                g_ui16T5TReadIndex += ui8RxLength - ui8LocalReadIndex;


                            }
                            else
                            {
                                // Store Incoming NDEF Message into g_pui8T5TBuffer
                                if(g_ui16T5TMaxNdefLen > (g_ui16T5TReadIndex + g_ui16T5TTLVRemaining))
                                {
                                    memcpy(&g_pui8T5TBuffer[g_ui16T5TReadIndex],&g_pui8T5TRxBuffer[ui8LocalReadIndex],g_ui16T5TTLVRemaining);

                                    //Serial_printBuffer((char *)&g_pui8T5TBuffer[0],(uint8_t) (g_ui16T5TReadIndex+g_ui16T5TTLVRemaining),RW_PAYLOAD_DATA);
                                }
                                else
                                {
                                    // NDEF message is larger than the size of the g_pui8T5TBuffer buffer
                                }

                                g_ui16T5TReadIndex += g_ui16T5TTLVRemaining;
                            }
                        }

                        // Incrementing Index

                        // If the current TLV remaining bytes are more than the available bytes
                        if(g_ui16T5TTLVRemaining > (ui8RxLength - ui8LocalReadIndex))
                        {
                            // Update Remaining
                            g_ui16T5TTLVRemaining -= (uint16_t) ( ui8RxLength - ui8LocalReadIndex);
                            // Increase Index to 16 to continue reading next block
                            ui8LocalReadIndex = ui8RxLength;
                        }
                        else
                        {
                            // Increase read index by remaining number of bytes
                            ui8LocalReadIndex += (uint8_t) g_ui16T5TTLVRemaining;
                            g_ui16T5TTLVRemaining = 0;
                            // Finished reading TLV
                            g_bT5TTLVSelected = false;
                        }
                    }
                }

                if(g_eT5TState != T5T_SELECTED_IDLE)
                {
                    // Incrment Block Number
                    g_ui16T5TBlockNumber++;

                    // EOF Not Found
                    if(g_ui16T5TBlockNumber > g_ui16T5TBlockCount)
                    {
                        //Serial_printf("Error: T5T Terminator TLV not found.",RW_STATUS_DATA);

                        // Print New Line
                        //Serial_printf("\n",RW_STATUS_DATA);

                        g_eT5TState = T5T_SELECTED_IDLE;
                    }
                }
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_FAIL)
            {
                g_eT5TState = T5T_SELECTED_IDLE;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_WRITE_NDEF:
            eT5TStatus = NFC_RW_T5T_getWriteSingleStatus();

            if(eT5TStatus == NFC_RW_T5T_CONNECTION_WRITE_SINGLE_SUCCESS)
            {
                g_ui16T5TBlockNumber++;
                g_ui16T5TNdefIndex = g_ui16T5TNdefIndex + 4;
                if(g_ui16T5TTLVRemaining > 4)
                {
                    g_ui16T5TTLVRemaining = g_ui16T5TTLVRemaining - 4;
                }
                else
                {
                    g_ui16T5TTLVRemaining = 0;
                    g_eT5TState = T5T_SELECTED_IDLE;

                    //Serial_printf("T5T Write Successful!",RW_STATUS_DATA);

                    // Print New Line
                    //Serial_printf("\n",RW_STATUS_DATA);

                    if (g_bT5TFormatting)
                    {
                        //Serial_printf("Re-present the Tag to Write to it.",RW_STATUS_DATA);

                        // Print New Line
                        //Serial_printf("\n",RW_STATUS_DATA);
                    }
                }
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_WRITE_SINGLE_FAIL)
            {
                if (NFC_RW_T5T_getT5TErrorCode() == 0x12)
                {
                    //Serial_printf("T5T Write Fail! Target Block Is Locked",RW_STATUS_DATA);
                }

                g_eT5TState = T5T_SELECTED_IDLE;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_SELECTED_IDLE:
            g_bT5TWaitForRsp = false;
            break;
        }
    }

    // Sending a command
    if(g_bT5TWaitForRsp == false)
    {
        switch(g_eT5TState)
        {
        case T5T_INVENTORY:
            if(NFC_RW_T5T_sendInventoryCmd(0x26,0x00,false) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_GET_SYS_INFO_EXT:
            if(NFC_RW_T5T_sendGetSysInfoCmd(0x02,0x3B) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_GET_SYS_INFO_1:
            if(NFC_RW_T5T_sendGetSysInfoCmd(0x0A,0x2B) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_GET_SYS_INFO_2:
            if(NFC_RW_T5T_sendGetSysInfoCmd(0x02,0x2B) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_READ_CC:
            if(NFC_RW_T5T_sendReadSingleCmd(0x22,g_ui16T5TBlockNumber) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_READ_DATA:
            if(NFC_RW_T5T_sendReadSingleCmd(0x22,g_ui16T5TBlockNumber) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_READ_NDEF:
            if(NFC_RW_T5T_sendReadSingleCmd(0x22,g_ui16T5TBlockNumber) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_WRITE_NDEF:
            if(g_ui16T5TTLVRemaining > 0)
            {
                if(NFC_RW_T5T_sendWriteSingleCmd(T5T_REQ_FLAG_HIGH_DATA|T5T_REQ_FLAG_OPTION,g_ui16T5TBlockNumber,&g_pui8T5TBuffer[g_ui16T5TNdefIndex],4) == STATUS_SUCCESS)
                {
                    g_bT5TWaitForRsp = true;
                }
                else
                {
                    g_eT5TState = T5T_SELECTED_IDLE;
                }
            }
            break;
        case T5T_SELECTED_IDLE:
            break;
        }
    }
}



/*
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        UART_transmitData(EUSCI_A0_BASE, MAP_UART_receiveData(EUSCI_A0_BASE));
    }

}*/
void printString(char output[])
{
    int size = sizeof output / sizeof output[0];
    int i;
    for(i = 0; i < size; i++)
    {
        UART_transmitData(EUSCI_A0_BASE, output[i]);
    }
}
//*****************************************************************************
//
//! NFC_configuration - Handles the initial NFC configuration.
//!
//! Setup all NFC Mode Parameters.
//!
//! Current modes enabled: Card Emulation
//! Current modes supported: Card Emulation and Peer 2 Peer
//! Reader/Writer is NOT supported yet.
//!
//*****************************************************************************

void NFC_configuration(void)
{
#if NFC_READER_WRITER_ENABLED
    g_sRWSupportedModes.ui8byte = 0x00;
    g_sRWSupportedBitrates.ui16byte = 0x0000;
    g_sRWSetupOptions.ui16byte = 0x0000;
#endif

    // Set the TRF7970 Version being used
    g_eTRFVersion = TRF7970_A;

    // External Amplifer (disconnected by default)
    g_bExtAmplifier = false;

    // Configure TRF External Amplifier for the transceiver
    TRF79x0_setExtAmplifer(g_bExtAmplifier);

    // Configure TRF Power Supply (5V = true, 3V = false)

    g_bTRF5VSupply = false;


    // Configure TRF Power Supply
    TRF79x0_setPowerSupply(g_bTRF5VSupply);

    // Milliseconds the NFC stack will be in listen mode
    g_ui16ListenTime = 500;

    // Set the time the NFC stack will be with the RF field disabled (listen mode)
    NFC_setListenTime(g_ui16ListenTime);

    // Enable (1) or disable (0) the Auto SDD Anti-collision function of the TRF7970A
    g_bEnableAutoSDD = 0;

#if NFC_READER_WRITER_ENABLED
    // Enable Reader Writer Supported Modes
    g_sRWSupportedModes.bits.bNfcA = 1;
    g_sRWSupportedModes.bits.bNfcB = 1;
    g_sRWSupportedModes.bits.bNfcF = 1;
    g_sRWSupportedModes.bits.bISO15693 = 1;

    // NFC-A Bitrates
    g_sRWSupportedBitrates.bits.bNfcA_106kbps = 1;      // Must be enabled if bNfcA is set
    g_sRWSupportedBitrates.bits.bNfcA_212kbps = 0;
    g_sRWSupportedBitrates.bits.bNfcA_424kbps = 0;
    g_sRWSupportedBitrates.bits.bNfcA_848kbps = 1;
    // NFC-B Bitrates
    g_sRWSupportedBitrates.bits.bNfcB_106kbps = 1;      // Must be enabled if bNfcB is set
    g_sRWSupportedBitrates.bits.bNfcB_212kbps = 0;
    g_sRWSupportedBitrates.bits.bNfcB_424kbps = 0;
    g_sRWSupportedBitrates.bits.bNfcB_848kbps = 1;
    // NFC-F Bitrates
    g_sRWSupportedBitrates.bits.bNfcF_212kbps = 1;
    g_sRWSupportedBitrates.bits.bNfcF_424kbps = 1;
    // ISO15693 Bitrates
    g_sRWSupportedBitrates.bits.bISO15693_26_48kbps = 1;

    // Default Max number of WTX 2
    g_sRWSetupOptions.bits.ui3RWMaxWTX = 2;
    // Default Max number of ACK 2
    g_sRWSetupOptions.bits.ui3RWMaxACK = 2;
    // Default Max number of NACK 2
    g_sRWSetupOptions.bits.ui3RWMaxNACK = 2;
    // Default Max number of DSL 2
    g_sRWSetupOptions.bits.ui3RWMaxDSL = 2;

    g_ui8IsoDepInitiatorDID = 0x00;
#endif



#if NFC_READER_WRITER_ENABLED
    if (g_sRWSupportedModes.ui8byte != 0x00)
    {
        // To be made shortly
        NFC_RW_configure(g_sRWSupportedModes,g_sRWSupportedBitrates);
    }
    ISODEP_configure_RW(g_sRWSetupOptions,g_ui8IsoDepInitiatorDID);
#endif

    // Set the Auto SDD flag within nfc_a.c
    NFC_A_setAutoSDD(g_bEnableAutoSDD);

    // Set the current TRF version within trf79x0.c
    TRF79x0_setVersion(g_eTRFVersion);

    // Set Certification Support for all Protocols - Required for NFC Forum Certification
    NFC_setSupportCertification(g_bSupportCertification);

    // Set Test Enable flag within trf79x0.c - Required for NFC Forum Certification
    TRF79x0_testFlag(g_bSupportCertification);

}

//*****************************************************************************
//
//! Serial_processCommand - Process incoming commands from NFC host.
//!
//! Checks for the SOF (0xFE) from the host, and processes the commands. The
//! possible commands are:
//!     COMMUNICATION_START - NFC Host connected
//!     START_P2P_CMD - Enable the P2P modes included in the command
//!     STOP_P2P_CMD - Disable the P2P stack.
//!     ACK_CMD - Not currently used
//!     NDEF_PAYLOAD - NDEF data to send via the P2P stack.
//!     COMMUNICATION_END - Disconnect form the NFC Host.
//!
//! \return None.
//
//*****************************************************************************
void Serial_processCommand(void)
{

    tNFCControllerCommands eHostCommand;
#if NFC_READER_WRITER_ENABLED
    t_sNfcRWMode sRWMode;
    t_sNfcRWCommBitrate sRWBitrate;
#endif
    uint8_t ui8CurrentConfiguration[19];

    // When SOF and length are received, but still missing data
    if(g_ui16BytesReceived > 2 && ((g_ui8SerialBuffer[2] + 3) > g_ui16BytesReceived))
    {
        // Wait until full packet has been received
    }
    // Waiting for Length Byte
    else if(g_ui16BytesReceived < 3)
    {
        // Wait until Length Byte received
    }
    else if(g_ui8SerialBuffer[2] + 3 == g_ui16BytesReceived)    // Length
    {
        eHostCommand = (tNFCControllerCommands) g_ui8SerialBuffer[1];
        if(eHostCommand == COMM_START && g_ui16BytesReceived == 3)
        {
            // Initialize the Current Configuration variables to zero
            //
            memset(ui8CurrentConfiguration,0x00,19);

            // Turn on LED
            g_bSerialConnectionEstablished = true;
            NFC_HOST_LED_POUT |= NFC_HOST_LED_BIT;
            //Serial_printf(NFC_FW_VERSION,FW_VERSION_CMD);

#if NFC_READER_WRITER_ENABLED
            ui8CurrentConfiguration[0] |= READER_WRITER_FW_ENABLED;
            ui8CurrentConfiguration[4] |= g_sRWSupportedModes.ui8byte;
            ui8CurrentConfiguration[5] |= (uint8_t) (g_sRWSupportedBitrates.ui16byte & 0xFF);
            ui8CurrentConfiguration[6] |= (uint8_t) ((g_sRWSupportedBitrates.ui16byte >> 8) & 0xFF);
#endif

            ui8CurrentConfiguration[7] = (uint8_t) (g_ui16ListenTime & 0xFF);
            ui8CurrentConfiguration[8] = (uint8_t) ((g_ui16ListenTime >> 8) & 0xFF);

            ui8CurrentConfiguration[9] = g_bSupportCertification;

#if NFC_READER_WRITER_ENABLED
            ui8CurrentConfiguration[12] = (uint8_t) (g_sRWSetupOptions.ui16byte & 0xFF);

            ui8CurrentConfiguration[13] = (uint8_t) ((g_sRWSetupOptions.ui16byte >> 8) & 0xFF);

            ui8CurrentConfiguration[14] = g_ui8IsoDepInitiatorDID;
#endif

            ui8CurrentConfiguration[15] = g_bEnableAutoSDD;

            ui8CurrentConfiguration[16] = g_bExtAmplifier;

            ui8CurrentConfiguration[17] = g_bTRF5VSupply;

            ui8CurrentConfiguration[18] = (uint8_t) g_eTRFVersion;

            //Serial_printBuffer((char *)ui8CurrentConfiguration,19,NFC_CONFIGURATION);

        }
        else if(eHostCommand == COMM_END && g_ui16BytesReceived == 3)
        {
            // Turn off LED
            NFC_HOST_LED_POUT &= ~NFC_HOST_LED_BIT;
            g_bSerialConnectionEstablished = false;
        }
        else if(eHostCommand == NFC_TEST_CONFIG && g_ui16BytesReceived == 9)
        {
            // NFC Certification
            g_bSupportCertification = g_ui8SerialBuffer[3];

            // Set Certification Support for all Protocols - Required for NFC Forum Certification
            NFC_setSupportCertification(g_bSupportCertification);

            // Set Test Enable flag within trf79x0.c - Required for NFC Forum Certification
            TRF79x0_testFlag(g_bSupportCertification);


 #if NFC_READER_WRITER_ENABLED
            // RW Options
            g_sRWSetupOptions.ui16byte =  (uint16_t) (g_ui8SerialBuffer[6]) + ((uint16_t) g_ui8SerialBuffer[7] << 8);

            // ISO DEP DID
            g_ui8IsoDepInitiatorDID = g_ui8SerialBuffer[8];

            ISODEP_configure_RW(g_sRWSetupOptions,g_ui8IsoDepInitiatorDID);
#endif
        }
        else if(eHostCommand == TRF_SETTINGS && g_ui16BytesReceived == 9)
        {
            // TRF Version Number
            g_eTRFVersion = (tTRF79x0_Version) g_ui8SerialBuffer[3];

            // Set the current TRF version within trf79x0.c
            TRF79x0_setVersion(g_eTRFVersion);

            // Listen Time
            g_ui16ListenTime = (uint16_t) (g_ui8SerialBuffer[4]) + ((uint16_t) g_ui8SerialBuffer[5] << 8);

            // Set the time the NFC stack will be with the RF field disabled (listen mode)
            NFC_setListenTime(g_ui16ListenTime);

            // 5V Power Supply
            g_bTRF5VSupply = g_ui8SerialBuffer[6];

            // Configure TRF Power Supply
            TRF79x0_setPowerSupply(g_bTRF5VSupply);

            // External Amplifier
            g_bExtAmplifier = g_ui8SerialBuffer[7];

            // Configure TRF External Amplifier for the transceiver
            TRF79x0_setExtAmplifer(g_bExtAmplifier);

            // Auto-SDD Setting
            g_bEnableAutoSDD = g_ui8SerialBuffer[8];

            // Set the Auto SDD flag within nfc_a.c
            NFC_A_setAutoSDD(g_bEnableAutoSDD);
        }
#if (NFC_READER_WRITER_ENABLED)
        else if(eHostCommand == RW_START_CMD && g_ui16BytesReceived == 6)
        {
            g_sRWSupportedBitrates.ui16byte = (g_ui8SerialBuffer[4] << 8) + g_ui8SerialBuffer[3];
            g_sRWSupportedModes.ui8byte = g_ui8SerialBuffer[5];

            NFC_RW_configure(g_sRWSupportedModes,g_sRWSupportedBitrates);
        }
        else if(eHostCommand == RW_STOP_CMD && g_ui16BytesReceived == 3)
        {
            NFC_RW_disable();
            g_sRWSupportedModes.ui8byte = 0x00;
        }
        else if(eHostCommand == RW_WRITE_TAG)
        {
            if(NFC_RW_getModeStatus(&sRWMode,&sRWBitrate))
            {
                if( sRWMode.bits.bNfcA == 1)
                {
                    if(NFC_A_getSAK() == 0x00)
                    {
                        // T2T Tag Write State Machine
                        T2T_writeNDEF(&g_ui8SerialBuffer[3],(uint16_t) (g_ui16BytesReceived-3));
                    }
                    else if(NFC_A_getSAK() & 0x20)
                    {
                        // T4T Tag Write State Machine
                        T4T_writeNDEF(&g_ui8SerialBuffer[3],(uint16_t) (g_ui16BytesReceived-3));
                    }
                }
                else if(sRWMode.bits.bNfcB == 1)
                {
                    if(NFC_B_isISOCompliant())
                    {
                        // T4T Tag Write State Machine
                        T4T_writeNDEF(&g_ui8SerialBuffer[3],(uint16_t) (g_ui16BytesReceived-3));
                    }
                }
                else if(sRWMode.bits.bNfcF == 1)
                {
                    // T3T Tag State Machine
                    T3T_writeNDEF(&g_ui8SerialBuffer[3],(uint16_t) (g_ui16BytesReceived-3));
                }
                else if(sRWMode.bits.bISO15693 == 1)
                {
                    // T5T Tag State Machine
                    T5T_writeNDEF(&g_ui8SerialBuffer[3],(uint16_t) (g_ui16BytesReceived-3));
                }
            }
        }
        else if (eHostCommand == RW_FORMAT_TAG)
        {
            if(NFC_RW_getModeStatus(&sRWMode,&sRWBitrate))
            {
                if(sRWMode.bits.bISO15693 == 1)
                {
                    T5T_formatTag();
                }
                else
                {
                    //Serial_printf("Cannot Format non-T5T Tags! \n",RW_STATUS_DATA);
                }
            }

        }
#endif
        g_ui16BytesReceived = 0;
    }
    else
    {
        // Partial Command Received
        g_ui16BytesReceived = 0x00;
    }
}


//*****************************************************************************
//
//! LCD_init - Initialize the LCD drivers and display CE code status.
//!
//! Initialize LCD - Initialize the LCD drivers, display TI logo for 1 second,
//! then display the NFC logo for 1 second, and display the version of the CE
//! code.
//!
//! \return None.
//
//*****************************************************************************
void LCD_init(void)
{

}


//*****************************************************************************
//
//! updateLcdfcStatus - Update the current P2P mode information.
//!
//! \param bUpdateRssiOnly defines if the function should update the RSSI value
//! only. When it is set to false, the P2P mode, bitrate, and number of bytes
//! received are also updated on the LCD.
//!
//! Update the activated P2P mode, bitrate, and number of bytes received.
//!
//! \return None.
//
//*****************************************************************************
void updateLcdfcStatus(bool bUpdateRssiOnly)
{
    static uint8_t ui8RssiValue = 0x00;
    char pui8Buffer[3];
    char pcUIDString[30];
    tTRF79x0_Status eTrfStatus;

    uint8_t * pcDataBuffer;
    uint8_t ui8UIDLen;
    uint8_t ui8Temp;

    eTrfStatus = TRF79x0_getStatus();

    if(bUpdateRssiOnly == false)
    {
        // Reset RSSI Value when the other parameters are updated
        ui8RssiValue = 0x00;

        if(eTrfStatus.eCurrentMode == TARGET_MODE )
        {
            if(eTrfStatus.sTargetMode.bits.bPassiveTypeA == 1)
            {
                if(eTrfStatus.bNfcModeEnabled == true)
                {
                    //Serial_printf("Target",P2P_MODE_DATA);
                    //Serial_printf("Passive A",P2P_TECH_DATA);;
                }
                else
                {

                    //Serial_printf("T4T",CE_TYPE_DATA);
                    //Serial_printf("NFC-A",CE_TECH_DATA);
                    //Serial_printf("106 kbps",CE_BITR_DATA);

                    // Get NFC A ID Length
                    NFC_A_getNfcAId(&pcDataBuffer,&ui8UIDLen);

                    for(ui8Temp=0;ui8Temp<ui8UIDLen;ui8Temp++)
                    {
                        // Convert hex values to ascii
                        //convertByteToAscii(pcDataBuffer[ui8Temp],(uint8_t *) pui8Buffer);

                        pcUIDString[ui8Temp*2] = pui8Buffer[0];
                        pcUIDString[(ui8Temp*2)+1] = pui8Buffer[1];
                    }
                    // Set last byte has null character
                    pcUIDString[ui8UIDLen*2] = 0x00;

                    //Serial_printf(pcUIDString,CE_UID_DATA);
                }

            }
            else if(eTrfStatus.sTargetMode.bits.bPassiveTypeB == 1)
            {

                //Serial_printf("T4T",CE_TYPE_DATA);
                //Serial_printf("NFC-B",CE_TECH_DATA);
                //Serial_printf("106 kbps",CE_BITR_DATA);

                // Get NFC B ID Length
                NFC_B_getNfcBId(&pcDataBuffer, &ui8UIDLen);

                for(ui8Temp=0;ui8Temp<ui8UIDLen;ui8Temp++)
                {
                    // Convert hex values to ascii
                    //convertByteToAscii(pcDataBuffer[ui8Temp],(uint8_t *) pui8Buffer);

                    pcUIDString[ui8Temp*2] = pui8Buffer[0];
                    pcUIDString[(ui8Temp*2)+1] = pui8Buffer[1];
                }
                // Set last byte has null character
                pcUIDString[ui8UIDLen*2] = 0x00;

                //Serial_printf(pcUIDString,CE_UID_DATA);
            }
            else if(eTrfStatus.sTargetMode.bits.bPassiveTypeF == 1 && eTrfStatus.bNfcModeEnabled == true)
            {

                //Serial_printf("Target",P2P_MODE_DATA);
                //Serial_printf("Passive F",P2P_TECH_DATA);;
            }
            else if(eTrfStatus.sTargetMode.bits.bActiveTypeA == 1 && eTrfStatus.bNfcModeEnabled == true)
            {

                //Serial_printf("Target",P2P_MODE_DATA);
                //Serial_printf("Active A",P2P_TECH_DATA);;
            }
            else if(eTrfStatus.sTargetMode.bits.bActiveTypeF == 1 && eTrfStatus.bNfcModeEnabled == true)
            {


                //Serial_printf("Target",P2P_MODE_DATA);
                //Serial_printf("Active F",P2P_TECH_DATA);;
            }

            if(eTrfStatus.sTargetPayloadFrequency.bits.b106kpbs == 1)
            {
                if(eTrfStatus.bNfcModeEnabled == true)
                {
                    //Serial_printf("106 kbps",P2P_BITR_DATA);
                }

            }
            else if(eTrfStatus.sTargetPayloadFrequency.bits.b212kpbs == 1)
            {
                if(eTrfStatus.bNfcModeEnabled == true)
                {
                    //Serial_printf("212 kbps",P2P_BITR_DATA);
                }

            }
            else if(eTrfStatus.sTargetPayloadFrequency.bits.b424kpbs == 1)
            {
                if(eTrfStatus.bNfcModeEnabled == true)
                {
                    //Serial_printf("424 kbps",P2P_BITR_DATA);
                }

            }
        }
        else if(eTrfStatus.eCurrentMode  == INITIATOR_MODE)
        {
            if(eTrfStatus.sInitiatorMode.bits.bPassiveTypeA == 1)
            {

                if(eTrfStatus.bNfcModeEnabled == true)
                {


                    //Serial_printf("Initiator",P2P_MODE_DATA);
                    //Serial_printf("Passive A",P2P_TECH_DATA);;
                }
                else
                {
                    //Serial_printf("NFC-A",RW_TECH_DATA);

                    // Get NFC A ID Length
                    NFC_A_getNfcAId(&pcDataBuffer,&ui8UIDLen);

                    for(ui8Temp=0;ui8Temp<ui8UIDLen;ui8Temp++)
                    {
                        // Convert hex values to ascii
                        //convertByteToAscii(pcDataBuffer[ui8Temp],(uint8_t *) pui8Buffer);

                        pcUIDString[ui8Temp*2] = pui8Buffer[0];
                        pcUIDString[(ui8Temp*2)+1] = pui8Buffer[1];
                    }
                    // Set last byte has null character
                    pcUIDString[ui8UIDLen*2] = 0x00;

                    // Read SAK Value
                    ui8Temp = NFC_A_getSAK();

                    if(ui8Temp & 0x20)
                    {
                        //Serial_printf("T4T",RW_TYPE_DATA);
                    }
                    else
                    {
                        //Serial_printf("T2T",RW_TYPE_DATA);
                    }

                    //Serial_printf(pcUIDString,RW_UID_DATA);


                }
            }
            else if(eTrfStatus.sInitiatorMode.bits.bPassiveTypeB == 1)
            {
                //Serial_printf("NFC-B",RW_TECH_DATA);

                // Get NFC B ID Length
                NFC_B_getNfcBId(&pcDataBuffer, &ui8UIDLen);

                for(ui8Temp=0;ui8Temp<ui8UIDLen;ui8Temp++)
                {
                    // Convert hex values to ascii
                    //convertByteToAscii(pcDataBuffer[ui8Temp],(uint8_t *) pui8Buffer);

                    pcUIDString[ui8Temp*2] = pui8Buffer[0];
                    pcUIDString[(ui8Temp*2)+1] = pui8Buffer[1];
                }
                // Set last byte has null character
                pcUIDString[ui8UIDLen*2] = 0x00;

                //Serial_printf("T4T",RW_TYPE_DATA);

                //Serial_printf(pcUIDString,RW_UID_DATA);

            }
            else if(eTrfStatus.sInitiatorMode.bits.bPassiveTypeF == 1)
            {
                if(eTrfStatus.bNfcModeEnabled == true)
                {

                    //Serial_printf("Initiator",P2P_MODE_DATA);
                    //Serial_printf("Passive F",P2P_TECH_DATA);;
                }
                else
                {
                    //Serial_printf("NFC-F",RW_TECH_DATA);

                    // Get NFC F ID Length
                    NFC_F_getNFCID2(&pcDataBuffer, &ui8UIDLen);

                    for(ui8Temp=0;ui8Temp<ui8UIDLen;ui8Temp++)
                    {
                        // Convert hex values to ascii
                        //convertByteToAscii(pcDataBuffer[ui8Temp],(uint8_t *) pui8Buffer);

                        pcUIDString[ui8Temp*2] = pui8Buffer[0];
                        pcUIDString[(ui8Temp*2)+1] = pui8Buffer[1];
                    }
                    // Set last byte has null character
                    pcUIDString[ui8UIDLen*2] = 0x00;

                    //Serial_printf("T3T",RW_TYPE_DATA);

                    //Serial_printf(pcUIDString,RW_UID_DATA);

                }
            }
            else if(eTrfStatus.sInitiatorMode.bits.bActiveTypeA == 1 && eTrfStatus.bNfcModeEnabled == true)
            {

                //Serial_printf("Initiator",P2P_MODE_DATA);
                //Serial_printf("Active A",P2P_TECH_DATA);;
            }
            else if(eTrfStatus.sInitiatorMode.bits.bActiveTypeF == 1 && eTrfStatus.bNfcModeEnabled == true)
            {

                //Serial_printf("Initiator",P2P_MODE_DATA);
                //Serial_printf("Active F",P2P_TECH_DATA);;
            }
            else if(eTrfStatus.sInitiatorMode.bits.bPassive15693 == 1)
            {
                //Serial_printf("NFC-V",RW_TECH_DATA);

                // Get NFC V ID Length
                NFC_RW_T5T_getT5TUID(&pcDataBuffer,&ui8UIDLen);

                for(ui8Temp=ui8UIDLen;ui8Temp>0;ui8Temp--)
                {
                    // Convert hex values to ascii
                    //convertByteToAscii(pcDataBuffer[ui8Temp-1],(uint8_t *) pui8Buffer);

                    pcUIDString[(ui8UIDLen-ui8Temp)*2] = pui8Buffer[0];
                    pcUIDString[((ui8UIDLen-ui8Temp)*2)+1] = pui8Buffer[1];
                }
                // Set last byte has null character
                pcUIDString[ui8UIDLen*2] = 0x00;

                //Serial_printf("T5T",RW_TYPE_DATA);

                //Serial_printf(pcUIDString,RW_UID_DATA);


            }

            if(eTrfStatus.sInitiatorPayloadFrequency.bits.b26_48_kbps_1_out_4 == 1)
            {


                //Serial_printf("26.48 kbps",RW_BITR_DATA);
            }
            else if(eTrfStatus.sInitiatorPayloadFrequency.bits.b106kpbs == 1)
            {
                if(eTrfStatus.bNfcModeEnabled == true)
                {
                    //Serial_printf("106 kbps",P2P_BITR_DATA);
                }
                else
                {
                    //Serial_printf("106 kbps",RW_BITR_DATA);
                }

            }
            else if(eTrfStatus.sInitiatorPayloadFrequency.bits.b212kpbs == 1)
            {
                if(eTrfStatus.bNfcModeEnabled == true)
                {
                    //Serial_printf("212 kbps",P2P_BITR_DATA);
                }
                else
                {
                    //Serial_printf("212 kbps",RW_BITR_DATA);
                }

            }
            else if(eTrfStatus.sInitiatorPayloadFrequency.bits.b424kpbs == 1)
            {
                if(eTrfStatus.bNfcModeEnabled == true)
                {
                    //Serial_printf("424 kbps",P2P_BITR_DATA);
                }
                else
                {
                    //Serial_printf("424 kbps",RW_BITR_DATA);
                }

            }
            else if(eTrfStatus.sInitiatorPayloadFrequency.bits.b848kpbs == 1)
            {


                //Serial_printf("848 kbps",RW_BITR_DATA);
            }

        }


        ui8RssiValue = eTrfStatus.ui8InternalRssi;
        //convertByteToAscii(eTrfStatus.ui8InternalRssi,(uint8_t *)pui8Buffer);

        if(eTrfStatus.bNfcModeEnabled == true)
        {
            //Serial_printf(pui8Buffer,P2P_RSSI_DATA);
        }
        else
        {
            //Serial_printf(pui8Buffer,RW_RSSI_DATA);
            //Serial_printf(pui8Buffer,CE_RSSI_DATA);
        }
    }

    if(eTrfStatus.ui8InternalRssi != ui8RssiValue)
    {
        ui8RssiValue = eTrfStatus.ui8InternalRssi;
        //convertByteToAscii(eTrfStatus.ui8InternalRssi,(uint8_t *)pui8Buffer);

        if(eTrfStatus.bNfcModeEnabled == true)
        {
            //Serial_printf(pui8Buffer,P2P_RSSI_DATA);
        }
        else
        {
            //Serial_printf(pui8Buffer,RW_RSSI_DATA);
            //Serial_printf(pui8Buffer,CE_RSSI_DATA);
        }
    }
}

/*
* EUSCI A0 UART interrupt handler. Receives data from GUI and sets LED color/blink frequency
*/
void EusciA0_ISR(void)
{
   uint8_t ui8RxByte = UCA0RXBUF;

   g_ui8SerialBuffer[g_ui16BytesReceived] = ui8RxByte;

   // Check if we are receiving a packet
   if(g_ui16BytesReceived > 0)
   {
       g_ui16BytesReceived++;
   }
   // Check for Start of Frame
   else if(ui8RxByte == 0xFE)
   {
       g_ui16BytesReceived++;
   }
   else
   {
       // Do nothing
   }
}


/*
 * ASYNCHRONOUS EVENT HANDLERS -- Start
 */

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
        transmitString(" [WLAN EVENT] NULL Pointer Error \n\r");

    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                transmitString(" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                transmitString(" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            transmitString(" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
        transmitString(" [NETAPP EVENT] NULL Pointer Error \n\r");

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /*
             * Information about the connected AP's IP, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */
        }
        break;

        default:
        {
            transmitString(" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pHttpEvent - Contains the relevant event information
    \param[in]      pHttpResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
    transmitString(" [HTTP EVENT] Unexpected event \n\r");
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    transmitString(" [GENERAL EVENT] \n\r");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
        transmitString(" [SOCK EVENT] NULL Pointer Error \n\r");

    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        {
            /*
            * TX Failed
            *
            * Information about the socket descriptor and status will be
            * available in 'SlSockEventData_t' - Applications can use it if
            * required
            *
            * SlSockEventData_t *pEventData = NULL;
            * pEventData = & pSock->EventData;
            */
            switch( pSock->EventData.status )
            {
                case SL_ECLOSE:
                    transmitString(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
                break;


                default:
                    transmitString(" [SOCK EVENT] Unexpected event \n\r");
                break;
            }
        }
        break;

        default:
            transmitString(" [SOCK EVENT] Unexpected event \n\r");
        break;
    }
}
/*
 * ASYNCHRONOUS EVENT HANDLERS -- End
 */




static void generateUniqueID() {
    CRC32_setSeed(TLV->RANDOM_NUM_1, CRC32_MODE);
    CRC32_set32BitData(TLV->RANDOM_NUM_2);
    CRC32_set32BitData(TLV->RANDOM_NUM_3);
    CRC32_set32BitData(TLV->RANDOM_NUM_4);
    int i;
    for (i = 0; i < 6; i++)
    CRC32_set8BitData(macAddressVal[i], CRC32_MODE);

    uint32_t crcResult = CRC32_getResult(CRC32_MODE);
    sprintf(uniqueID, "%06X", crcResult);
}

//****************************************************************************
//
//!    \brief MQTT message received callback - Called when a subscribed topic
//!                                            receives a message.
//! \param[in]                  data is the data passed to the callback
//!
//! \return                        None
//
//****************************************************************************
static void messageArrived(MessageData* data) {
    char buf[BUFF_SIZE];

    // Check for buffer overflow
    if (data->topicName->lenstring.len >= BUFF_SIZE) {
//      UART_PRINT("Topic name too long!\n\r");
        return;
    }
    if (data->message->payloadlen >= BUFF_SIZE) {
//      UART_PRINT("Payload too long!\n\r");
        return;
    }


    strncpy(buf, data->topicName->lenstring.data,
        min(BUFF_SIZE, data->topicName->lenstring.len));
    buf[data->topicName->lenstring.len] = 0;



    strncpy(buf, data->message->payload,
        min(BUFF_SIZE, data->message->payloadlen));
    buf[data->message->payloadlen] = 0;



    struct controllerData_t tempData = parseControllerJSON(buf, 10);
    transmitString("Pressed: ");
    transmitInt(tempData.pressed);
    transmitString("Key: ");
    transmitString(tempData.key);
    transmitString("\n\r");



    //transmitString((unsigned char*)buf);
    //transmitString("\n\r");

    return;
}

/*
 * Port 1 interrupt handler. This handler is called whenever the switch attached
 * to P1.1 is pressed.
 */
void PORT1_IRQHandler(void)
{
    uint32_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (status & GPIO_PIN1)
    {
        if (S1buttonDebounce == 0)
        {
            S1buttonDebounce = 1;

            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

            // Publish the unique ID
            publishID = 1;

            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
    if (status & GPIO_PIN4)
    {
        if (S2buttonDebounce == 0)
        {
            S2buttonDebounce = 1;

            transmitString(" MAC Address: \n\r ");
            transmitString(macStr);
            transmitString("\n\r");

            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        }
    }
}

void TA1_0_IRQHandler(void)
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    if (P1IN & GPIO_PIN1)
    {
        S1buttonDebounce = 0;
    }
    if (P1IN & GPIO_PIN4)
    {
        S2buttonDebounce = 0;
    }

    if ((P1IN & GPIO_PIN1) && (P1IN & GPIO_PIN4))
    {
        Timer_A_stopTimer(TIMER_A1_BASE);
    }
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


/*!
    \brief This function configure the SimpleLink device in its default state. It:
           - Sets the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregisters mDNS services
           - Remove all filters

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
*/
static _i32 configureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;

    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}

/*!
    \brief Connecting to a WLAN Access point

    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address

    \param[in]  None

    \return     0 on success, negative error-code on error

    \note

    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
*/
static _i32 establishConnectionWithAP()
{
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = PASSKEY_LEN;
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

/*!
    \brief This function initializes the application variables

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
static _i32 initializeAppVariables()
{
    g_Status = 0;
    pal_Memset(&g_AppData, 0, sizeof(g_AppData));

    return SUCCESS;
}

/*!
    \brief This function displays the application's banner

    \param      None

    \return     None
*/
static void displayBanner()
{
    transmitString("\n\r\n\r");
    transmitString(" MQTT Twitter Controlled RGB LED - Version ");
    transmitString(APPLICATION_VERSION);
    transmitString("\n\r*******************************************************************************\n\r");
}

