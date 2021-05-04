/*
 * main_helper.c
 *
 *  Created on: Apr 30, 2021
 *      Author: wills
 */

#include "main.h"


void roverInit()
{
    //rslk rover init
    initRSLKRover(SYS_CLK);
    initRSLKTimer32(TIMER32_1_BASE);
    enableWheel(&right_wheel_data);
    setWheelDirForward(&right_wheel_data);
    enableWheel(&left_wheel_data);
    setWheelDirForward(&left_wheel_data);
    enableWheelCompensator(&left_wheel_data);
    enableWheelCompensator(&right_wheel_data);
    speed_state = SPEED_HOLD;
    speed = 100;
}

void positiveReaction()
{

}

void negativeReaction()
{
    rotateRightforTimeComp(NEG_REACTION_SPEED, 1000);
}

Tag_Type nfc_tag_detect(bool * tag_present, uint8_t * count)
{
    //transmitString("In NFC");transmitNewLine();
    Tag_Type ret_tag_type = NO_TAG;
    eTempNFCState = NFC_run();

    if(eTempNFCState == NFC_DATA_EXCHANGE_PROTOCOL)
    {

        if(*count != 2)
        {
            *count = 2;
        }
        transmitString("in exchange protocol\n\r");
        if(NFC_RW_getModeStatus(&sRWMode,&sRWBitrate))
        {
            NFC_RW_LED_POUT |= NFC_RW_LED_BIT;

            if( sRWMode.bits.bNfcA == 1)
            {
                if(NFC_A_getSAK() == 0x00)
                {
                    // T2T Tag State Machine
                    if(tag_present == false)
                    {
                        turnOn_LaunchpadLED2_green();
                        transmitString("Type 2 Tag Detect\n\r");
                        *tag_present = true;
                        ret_tag_type = NEGATIVE_TAG;
                        //turn 7 segment on to 0
                        //segmentWrite('b');
                       // msg7Seg.payload = "{\"sA\":0,\"sB\":0,\"sC\":1,\"sD\":1,\"sE\":1,\"sF\":1,\"sG\":1,\"sDP\":0}";
                        //msg7Seg.payloadlen = strlen(msg7Seg.payload);
                        //rc = MQTTPublish(&hMQTTClient, "XRTIC20/Feedback/SevenSegmentDisplay", &msg7Seg);


                        //msgRFID.payload = "{\"type\":2,\"effect\":\"None\"}";
                        //msgRFID.payloadlen = strlen(msgRFID.payload);
                        //rc = MQTTPublish(&hMQTTClient, "XRTIC20/Feedback/RFID", &msgRFID);
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
                if(*tag_present == false)
                {
                    turnOn_LaunchpadLED2_blue();
                    transmitString("Type 5 Tag Detect\n\r");
                    ret_tag_type = POSITIVE_TAG;
                    //turn 7 segment on to 0
                    //segmentWrite('a');
                    *tag_present = true;
                    //msg7Seg.payload = "{\"sA\":1,\"sB\":1,\"sC\":1,\"sD\":0,\"sE\":1,\"sF\":1,\"sG\":1,\"sDP\":0}";
                    //msg7Seg.payloadlen = strlen(msg7Seg.payload);
                    //rc = MQTTPublish(&hMQTTClient, "XRTIC20/Feedback/SevenSegmentDisplay", &msg7Seg);

                    ///msgRFID.payload = "{\"type\":5,\"effect\":\"None\"}";
                    //msgRFID.payloadlen = strlen(msgRFID.payload);;
                    //rc = MQTTPublish(&hMQTTClient, "XRTIC20/Feedback/RFID", &msgRFID);
                }
            }
        }
        else if(NFC_P2P_getModeStatus(&sP2PMode,&sP2PBitrate))
        {

        }
        else if(NFC_CE_getModeStatus(&sCEMode))
        {

        }
    }
    else
    {
        // Clear LEDs (RX & TX)
        if(*count != 0)
            *count--;

        if(*count == 0)
        {
           turnOff_LaunchpadLED1();
           turnOff_LaunchpadLED2_red();//LaunchpadLED2_green
           turnOff_LaunchpadLED2_green();
           turnOff_LaunchpadLED2_blue();

           NFC_RW_LED_POUT &= ~NFC_RW_LED_BIT;
           NFC_P2P_LED_POUT &= ~NFC_P2P_LED_BIT;
           NFC_CE_LED_POUT &= ~NFC_CE_LED_BIT;
           *tag_present = false;
        }

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


            // Initialize the RW T2T, T3T, T4T and T5 state machines
            T2T_init(g_ui8TxBuffer,256);
            T5T_init(g_ui8TxBuffer,256);
            //turn 7 segment on to 0
            //segmentWrite('0');
           // msg7Seg.payload = "{\"sA\":1,\"sB\":1,\"sC\":1,\"sD\":1,\"sE\":1,\"sF\":1,\"sG\":0,\"sDP\":0}";
            //msg7Seg.payloadlen = strlen(msg7Seg.payload);
            //rc = MQTTPublish(&hMQTTClient, "XRTIC20/Feedback/SevenSegmentDisplay", &msg7Seg);

            //buttonDebounce = 1;
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

    Serial_processCommand();
    return ret_tag_type;
}

void irInit()
{
    //SENSOR SET UP

    SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
    SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts
    I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
    OPT3101_Init();
    OPT3101_Setup();
    OPT3101_CalibrateInternalCrosstalk();
    OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
    StartTime = SysTick->VAL;
    OPT3101_StartMeasurement();
    //END IR INIT
}

void roboNav()
{
    transmitString("in RoboNav\n\r");
    transmitString("Distances: ");transmitNewLine();
    transmitString("D[0]: ");transmitInt(Distances[0]);transmitNewLine();
    transmitString("D[1]: ");transmitInt(Distances[1]);transmitNewLine();
    transmitString("D[2]: ");transmitInt(Distances[2]);transmitNewLine();
    uint16_t invalidDistance = INVALID_DISTANCE;
    uint16_t minDistance = MIN_DISTANCE;

    uint32_t bumpRoverRPM = BUMP_ROVER_RPM;
    uint32_t bumpRoverTurnDegrees = BUMP_ROVER_TURN_DEGREES;
    uint32_t bumpRoverBackwardsDist = BUMP_ROVER_BACKWARDS_MM;

    if((Distances[0] != invalidDistance && Distances[0] < minDistance) &&
        (Distances[2] != invalidDistance && Distances[2] < minDistance) &&
        (Distances[1] != invalidDistance && Distances[1] >= minDistance))
    {
        transmitString("case 1\n\r");
         moveForwardIndefinitelyComp(bumpRoverRPM);
    }
    else if((Distances[0] != invalidDistance && Distances[0] < minDistance))
    {
        transmitString("case 2\n\r");
         rotateRightIndefinitelyComp(bumpRoverRPM);
    }
    else if((Distances[2] != invalidDistance && Distances[2] < minDistance))
    {
        transmitString("case 3\n\r");
         rotateLeftIndefinitelyComp(bumpRoverRPM);
    }
    else if(bumpSensorPressed(BUMP0))
     {
         //moveBackwardForDistance(bumpRoverRPM, bumpRoverBackwardsDist);
         rotateLeftForDegreesComp(bumpRoverRPM, bumpRoverTurnDegrees);

     }
     else if(bumpSensorPressed(BUMP1))
     {
         //moveBackwardForDistance(bumpRoverRPM, bumpRoverBackwardsDist);

         rotateLeftForDegreesComp(bumpRoverRPM, bumpRoverTurnDegrees);
     }
     else if(bumpSensorPressed(BUMP2))
     {
         //moveBackwardForDistance(bumpRoverRPM, bumpRoverBackwardsDist);
         rotateLeftForDegreesComp(bumpRoverRPM, bumpRoverTurnDegrees);
     }
     else if(bumpSensorPressed(BUMP3))
     {
         //moveBackwardForDistance(bumpRoverRPM, bumpRoverBackwardsDist);
         rotateRightForDegreesComp(bumpRoverRPM, bumpRoverTurnDegrees);

     }
     else if(bumpSensorPressed(BUMP4))
     {
         //moveBackwardForDistance(bumpRoverRPM, bumpRoverBackwardsDist);
         rotateRightForDegreesComp(bumpRoverRPM, bumpRoverTurnDegrees);

     }
     else if(bumpSensorPressed(BUMP5))
     {
         //moveBackwardForDistance(bumpRoverRPM, bumpRoverBackwardsDist);
         rotateRightForDegreesComp(bumpRoverRPM, bumpRoverTurnDegrees);

     }
     else
     {
         moveForwardIndefinitelyComp(bumpRoverRPM);
     }

}
