#include "main.h"


#ifdef MSP432P401R_LAUNCHPAD_ENABLED
#include "lp_buttons.h"
#endif

#define DOGS102x6_DRAW_NORMAL 0x00
#define DOGS102x6_DRAW_INVERT 0x01

/*
 * GLOBAL VARIABLES -- Start
 */
/* Button debounce state variables */
volatile unsigned int S1buttonDebounce = 0;
volatile unsigned int S2buttonDebounce = 0;
volatile int publishID = 0;


int main(int argc, char** argv)
{

    // Initialize MCU
    MCU_init();
    MAP_WDT_A_holdTimer();
    initialize_LaunchpadLEDs();
    initUART();

    //Software Timers
    Init_Timer32_0(TIMER32_INIT_COUNT, CONTINUOUS); //0.1ms per count
    initSWTimer1();
    initSWTimer2();
    updateSW1WaitCycles(30000);
    //updateSW2WaitCycles(1000);

#if ROVER_ENABLE
    roverInit();
    bool rover_ctrl = true;
#endif

#if BUMP_ENABLE
    initBumpSensors();
#endif

#if SEG7_ENABLE
    initializeOutputs();//7seg
    //turn 7 segment on to 0
    segmentWrite('0');
#endif

#if NFC_ENABLE
    NFC_completeInit();

    g_ui16BytesReceived = 0x00;
    uint8_t count = 2;
    bool tag_present = false;
    Tag_Type tag_type;
    tag_type = NO_TAG;
#endif

#if MQTT_ENABLE
    int rc = 0;
    unsigned char buf[100];
    unsigned char readbuf[100];
    _i32 retVal = -1;

    retVal = initializeAppVariables();
    ASSERT_ON_ERROR(retVal);
    setUpMQTT(retVal, buf, readbuf, rc);
    button_two_interrupt_init();

    MQTTMessage msgRFID;
    MQTTMessageInit(&msgRFID);

    MQTTMessage msg7Seg;
    MQTTMessageInit(&msg7Seg);
#endif

#if IR_ENABLE
    uint32_t channel = 1;
    irInit();
#endif

#if GRIPPER_ENABLE
    Servo180 servoSettings;
    servo180InitArgs(&servoSettings,SYS_CLK,140,80,GPIO_PORT_P8,GPIO_PIN2);
    closeServo(&servoSettings);
#endif

#if (ROBONAV_ENABLE & MQTT_ENABLE)
    bool robonav_on = false;
#endif

    displayBanner();

    //message fore 7 segment
    //msg7Seg.payload = "{\"sA\":1,\"sB\":1,\"sC\":1,\"sD\":1,\"sE\":1,\"sF\":1,\"sG\":0,\"sDP\":0}";
    //msg7Seg.payloadlen = strlen(msg7Seg.payload);
    //rc = MQTTPublish(&hMQTTClient, "XRTIC20/Feedback/SevenSegmentDisplay", &msg7Seg);

    //Clock Check
    transmitString("MCLK: ");
    transmitInt(CS_getMCLK());
    SW_Timer_1.elapsedCycles = 0;

    MCU_delayMillisecond(100);

    while(1)
    {
#if ROVER_ENABLE
        //Rover updates
        checkEndCond();
        updateRoverState();
        compensator();
#endif

#if TELEM_ENABLE
        //NON-BLOCKING TELEMETRY SEND CHECK
        if(SW1TimerRollover())
        {
            //TRANSMIT ROVER

            //TRANSMIT BUMP

            //TRANSMIT IR

            //TRANSMIT RFID

            //TRANSMIT ROBONAV ON

            //test



            //SERVO DEMO
            //Comment either the toggle function
            //OR
            //Everything else in this if-statement
            //for a demonstration

            //toggleOpenClose(&servoSettings);
            //openServo(&servoSettings);

            /*if(!countDir)
                degree -= 5;
            else
                degree += 5;
            if(degree >= 145 || degree <= 80)
            {
                countDir = !countDir;
            }
            moveServoToDegree(degree, &servoSettings);*/
            //END SERVO DEMO
        }
        //END NON-BLOCKING TELEMETRY SEND CHECK
#endif

#if NFC_ENABLE
        tag_type = nfc_tag_detect(&tag_present, &count);

#if ROVER_ENABLE
        switch(tag_type)
        {
        case POSITIVE_TAG: break;
        case NEGATIVE_TAG: break;
        case NO_TAG: break;
        default: break;
        }
#endif

#endif

#if (NFC_ENABLE & MQTT_ENABLE)
        //NFC ENABLE STATE MACHINE
        if(recMQTTData.newData)
        {
            if(recMQTTData.pressed && !strcmp(recMQTTData.key, "space"))
            {
                recMQTTData.nfcEnabled = true;
                recMQTTData.newData = false;
            }
            else if(!recMQTTData.pressed && !strcmp(recMQTTData.key, "space"))
            {
                recMQTTData.nfcEnabled = false;
                recMQTTData.newData = false;
                //eTempNFCState == NFC_PROTOCOL_ACTIVATION;
            }
        }
        //END NFC ENABLE STATE MACHINE

        if(recMQTTData.nfcEnabled)
        {
            tag_type = nfc_tag_detect(&tag_present, &count);
        }

#if (ROVER_ENABLE & REACT_ENABLE)
        switch(tag_type)
        {
        case POSITIVE_TAG: break;
        case NEGATIVE_TAG:  rover_ctrl = false;
                            speed_state = HOLD;
                            initSWTimer2();
                            updateSW2WaitCycles(20000); //2seconds
                            negativeReaction();
                            break;
        case NO_TAG: break;
        default: break;
        }

        if(!rover_ctrl && SW2TimerRollover())
        {

        }
#endif
#endif

#if MQTT_ENABLE
        rc = MQTTYield(&hMQTTClient, 10);
        if (rc != 0) {
            transmitString(" MQTT failed to yield \n\r");
            LOOP_FOREVER();
        }

        //MessageData data1;
        //messageArrived(&data1);

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
#endif

#if GRIPPER_ENABLE
        //GRIPPER STATE MACHINE
        if(recMQTTData.newData)
        {
            if(recMQTTData.pressed)
            {
                if(!strcmp(recMQTTData.key,"w"))
                {
                    openServo(&servoSettings);
                    recMQTTData.newData = false;
                }
                else if(!strcmp(recMQTTData.key,"s"))
                {
                    closeServo(&servoSettings);
                    recMQTTData.newData = false;
                }

            }
        }
#endif

#if (ROVER_ENABLE & MQTT_ENABLE)
        //ROVER STATE MACHINE - BEGIN
        switch(speed_state)
        {
            case SPEED_INCREASE: speed += 10; /*transmitString("SPEED_INCREASE\n\r");*/break;
            case SPEED_DECREASE: speed -= 10; /*transmitString("SPEED_DECREASE\n\r");*/break;
            case SPEED_HOLD: /*transmitString("SPEED_HOLD\n\r");*/break;
            default: break;
        }
        //transmitInt(speed);transmitNewLine();
        if(!rover_ctrl)
        {
            if(recMQTTData.newData)
            {
                if(recMQTTData.pressed)
                {
                    //transmitString("Speed: ");
                    if(!strcmp(recMQTTData.key,"a"))
                    {
                        //transmitInt(speed);transmitNewLine();
                        speed_state = SPEED_INCREASE;
                        //recMQTTData.newData = false;
                    }
                    else if(!strcmp(recMQTTData.key,"d"))
                    {
                        //transmitInt(speed);transmitNewLine();
                        speed_state = SPEED_DECREASE;
                        //recMQTTData.newData = false;
                    }
                    else if(!strcmp(recMQTTData.key,"up"))
                    {
                        moveForwardIndefinitelyComp(speed);
                        recMQTTData.newData = false;
                        segmentWrite('1');
                    }
                    else if(!strcmp(recMQTTData.key,"down"))
                    {
                        moveBackwardIndefinitelyComp(speed);
                        recMQTTData.newData = false;
                        segmentWrite('2');
                    }
                    else if(!strcmp(recMQTTData.key,"right"))
                    {
                        rotateRightIndefinitelyComp(speed/8);
                        recMQTTData.newData = false;
                        segmentWrite('3');
                    }
                    else if(!strcmp(recMQTTData.key,"left"))
                    {
                        rotateLeftIndefinitelyComp(speed/8);
                        recMQTTData.newData = false;
                        segmentWrite('4');
                    }
                }
                else //released
                {
                    //rover state release logic
                    switch(rover_state)
                    {
                    case MOVING_FORWARD:    if(!strcmp(recMQTTData.key,"up"))
                                            {
                                                stopRover();
                                                //recMQTTData.newData = false;
                                            }break;
                    case MOVING_BACKWARD:   if(!strcmp(recMQTTData.key,"down"))
                                            {
                                                stopRover();
                                                //recMQTTData.newData = false;
                                            }break;
                    case ROTATING_RIGHT:    if(!strcmp(recMQTTData.key,"right"))
                                            {
                                                stopRover();
                                                //recMQTTData.newData = false;
                                            }break;
                    case ROTATING_LEFT:     if(!strcmp(recMQTTData.key,"left"))
                                            {
                                                stopRover();
                                                //recMQTTData.newData = false;
                                            }break;
                    default:
                        stopRover();
                        break;
                    }

                    //speed state release logics
                    switch(speed_state)
                    {
                    case SPEED_DECREASE:    if(!strcmp(recMQTTData.key,"d"))
                                            {
                                                speed_state = SPEED_HOLD;
                                            }break;
                    case SPEED_INCREASE:    if(!strcmp(recMQTTData.key,"a"))
                                            {
                                                speed_state = SPEED_HOLD;
                                            }break;
                    default: speed_state = SPEED_HOLD; break;
                    }

                    //recMQTTData.newData = false;
                }
                //segmentWrite('F');
                recMQTTData.newData = false;
            }
        }

#endif

#if BUMP_ENABLE
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
#endif

#if IR_ENABLE
        //if()
        OPT3101_StartMeasurementChannel(channel);
        if(channel < 2)
        {
            channel++;
        }
        else
        {
            channel = 0;
        }
        OPT3101_GetMeasurement(Distances,Amplitudes);

        transmitString("Left: ");
        transmitInt(Distances[0]);
        transmitString("mm | ");
        transmitString("Center: ");
        transmitInt(Distances[1]);
        transmitString("mm | ");
        transmitString("Right: ");
        transmitInt(Distances[2]);
        transmitString("mm\n\r");

#endif

#if (IR_ENABLE & NFC_ENABLE)
        //MINIMAL SENSOR STUFF
        if(!tag_present)
        {
            OPT3101_StartMeasurementChannel(channel);
            if(channel < 2)
            {
                channel++;
            }
            else
            {
                channel = 0;
            }
            OPT3101_GetMeasurement(Distances,Amplitudes);

            transmitString("Left: ");
            transmitInt(Distances[0]);
            transmitString("mm | ");
            transmitString("Center: ");
            transmitInt(Distances[1]);
            transmitString("mm | ");
            transmitString("Right: ");
            transmitInt(Distances[2]);
            transmitString("mm\n\r");
        }
        //END MINIMAL SENSOR STUFF
#endif

#if (IR_ENABLE & BUMP_ENABLE & ROBONAV_ENABLE & MQTT_ENABLE & ROVER_ENABLE)
        //enables with "robonav"

        if(recMQTTData.newData)
        {
            if(recMQTTData.pressed && !strcmp(recMQTTData.key, "rn"))
            {
                robonav_on = true;
                recMQTTData.newData = false;
            }
            else if(!recMQTTData.pressed && !strcmp(recMQTTData.key, "rn"))
            {
                robonav_on  = false;
                recMQTTData.newData = false;
                hardStop();
                //eTempNFCState == NFC_PROTOCOL_ACTIVATION;
            }
        }

        if(robonav_on)
            roboNav();
#endif

#if (IR_ENABLE & BUMP_ENABLE & ROBONAV_ENABLE & ROVER_ENABLE)
        roboNav();
#endif
        MCU_delayMillisecond(10);
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
//<<<<<<< HEAD
    g_ui16ListenTime = 10;//500;
//=======

//>>>>>>> branch 'master' of https://github.com/MattB99/XRTIC_Full_Integration.git

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
/*void LCD_init(void)
{

}*/


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
/*void updateLcdfcStatus(bool bUpdateRssiOnly)
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
}*/

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








