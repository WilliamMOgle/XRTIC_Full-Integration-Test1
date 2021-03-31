/*
 * mqtt.h
 *
 *  Created on: Mar 18, 2021
 *      Author: wills
 */

#ifndef MQTT_H_
#define MQTT_H_


#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "simplelink.h"
#include "sl_common.h"
#include "MQTTClient.h"
#include "uart_HAL.h"
#include "jsonParser.h"

/*
 * Values for below macros shall be modified per the access-point's (AP) properties
 * SimpleLink device will connect to following AP when the application is executed
 */

//Willi's Information
//#define SSID_NAME       "XRTIC-20"
//#define PASSKEY         ""
//#define SEC_TYPE        SL_SEC_TYPE_OPEN


/*
 * Matt's Information
 */

//#define SSID_NAME       "DuhFastStuff2.4"       // Access point name to connect to.
//#define PASSKEY         "bday0628"              //Password in case of secure AP

/*
 * Amar's friend Information
 */

//#define SSID_NAME       "VeeCastFox178"       // Access point name to connect to.
//#define PASSKEY         "VeeLearCar256!%"              //Password in case of secure AP

//Andrew's
#define SSID_NAME       "JeffECENo.1_5G-1"       // Access point name to connect to.
#define PASSKEY         "JeffNum1"              //Password in case of secure AP

/*
 * Amar's  Information
 */
//#define SSID_NAME       "XRTIC-20"
//#define PASSKEY         "wariokart"

#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2     /* Security type of the Access piont */
#define PASSKEY_LEN     pal_Strlen(PASSKEY)      /* Password length in case of secure AP */

/*
 * MQTT server and topic properties that shall be modified per application
 */
/*
#define MQTT_BROKER_SERVER  "192.168.1.2"
#define MQTT_BROKER_PORT 1883
#define MQTT_BROKER_USERNAME "admin"
#define MQTT_BROKER_PASSWORD "password"
*/
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
#define BUFF_SIZE 64


#define APPLICATION_VERSION "1.0.0"

#define MCLK_FREQUENCY 48000000
#define PWM_PERIOD 255

#define SL_STOP_TIMEOUT        0xFF

#define SMALL_BUF           32
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   1024

#define min(X,Y) ((X) < (Y) ? (X) : (Y))

char macStr[18];        // Formatted MAC Address String
char uniqueID[9];       // Unique ID generated from TLV RAND NUM and MAC Address
unsigned char macAddressVal[SL_MAC_ADDR_LEN];
struct controllerData_t recMQTTData;

Network n;

Client hMQTTClient;     // MQTT Client


struct{
    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
    _u8 SendBuff[MAX_SEND_BUF_SIZE];

    _u8 HostName[SMALL_BUF];
    _u8 CityName[SMALL_BUF];

    _u32 DestinationIP;
    _i16 SockID;
}g_AppData;


/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;


/*
 * STATIC FUNCTION DEFINITIONS -- Start
 */
 _i32 establishConnectionWithAP();
 _i32 configureSimpleLinkToDefaultState();
 _i32 initializeAppVariables();
 void displayBanner();
 void messageArrived(MessageData*);
 void generateUniqueID();
void setUpMQTT(_i32 retVal, unsigned char *buf, unsigned char *readbuf, int rc);
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent);
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent);
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,SlHttpServerResponse_t *pHttpResponse);
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent);
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock);
void MQTTMessageInit(MQTTMessage *msg);

#endif /* MQTT_H_ */
