/*
 * jsonParser.c
 *
 *  Created on: Sep 21, 2020
 *      Author: onemo
 */

#include "jsmn.h"
#include "jsonParser.h"

struct simpleData_t parseSimpleJSON(char *jsonString, int maxTokens)
{
    struct simpleData_t siD;
    siD.validData = validateChecksum(jsonString);

    if (!siD.validData)
    {
        return siD;
    }

    jsmn_parser parser;
    jsmn_init(&parser);
    jsmntok_t tokens[maxTokens];

    jsmn_parse(&parser, jsonString, strlen(jsonString), tokens, maxTokens);

    jsmntok_t key;
    unsigned int length;

    //set first value
    key = tokens[2];
    length = key.end - key.start;
    char keyString[length + 1];
    strncpy(keyString, &jsonString[key.start], length);
    keyString[length] = 0;
    siD.payloadID = atoi(keyString);

    //set second value
    key = tokens[4];
    length = key.end - key.start;
    char keyString2[length + 1];
    strncpy(keyString2, &jsonString[key.start], length);
    keyString2[length] = 0;
    siD.value = atoi(keyString2);

    return siD;
}

struct controllerData_t parseControllerJSON(char *jsonString, int maxTokens)
{
    struct controllerData_t cD;


    jsmn_parser parser;
    jsmn_init(&parser);


    jsmntok_t *tokens = malloc(maxTokens * sizeof(jsmntok_t));

    jsmn_parse(&parser, jsonString, strlen(jsonString), tokens, maxTokens);

    jsmntok_t key;
    unsigned int length;

    //set first value
    key = tokens[2];
    length = key.end - key.start;
    char *keyString = malloc((length + 1) * sizeof(char));
    strncpy(keyString, &jsonString[key.start], length);
    keyString[length] = '\0';
    cD.pressed = atoi(keyString);

    //set second value
    key = tokens[4];
    length = key.end - key.start;
    char *keyString2 = malloc((length + 1) * sizeof(char));
    strncpy(keyString2, &jsonString[key.start], length);
    keyString2[length] = '\0';
    cD.key = keyString2;

    free(tokens);

    return cD;
}



//assumes that returnedArray is size maxChars + 1
void convertSimpleDataToJSONString(struct simpleData_t simpleData,
                                   char *returnedArray, int maxChars)
{
    snprintf(returnedArray, maxChars, "{\"payloadID\":%d, \"value\":%d}",
             simpleData.payloadID, simpleData.value);

    returnedArray[maxChars] = '\0';
    char checksum = createChecksum(returnedArray);
    snprintf(returnedArray, maxChars, "%c{\"payloadID\":%d, \"value\":%d}",
             checksum, simpleData.payloadID, simpleData.value);
}


//assumes that returnedArray is size maxChars + 1
void convertSevSegToJSONString(struct sevSegData_t sevSegData,
                                   char *returnedArray, int maxChars)
{
    snprintf(returnedArray, maxChars, "{\"sA\":%d,\"sB\":%d,\"sC\":%d,\"sD\":%d,\"sE\":%d,\"sF\":%d,\"sG\":%d,\"sDP\":%d}",
             sevSegData.sA, sevSegData.sB, sevSegData.sC, sevSegData.sD, sevSegData.sE, sevSegData.sF, sevSegData.sG, sevSegData.sDP);

    returnedArray[maxChars] = '\0';

}

//assumes that returnedArray is size maxChars + 1
void convertSensorRFIDToJSONString(struct sensorRFIDData_t sensorRFIDData,
                                   char *returnedArray, int maxChars)
{
    snprintf(returnedArray, maxChars, "{\"type\":%s,\"effect\":%s}",
             sensorRFIDData.type, sensorRFIDData.effect);

    returnedArray[maxChars] = '\0';

}

//assumes that returnedArray is size maxChars + 1
void convertSensorBumpToJSONString(struct sensorBumpData_t sensorBumpData,
                                   char *returnedArray, int maxChars)
{
    snprintf(returnedArray, maxChars, "{\"b0\":%d,\"b1\":%d,\"b2\":%d,\"b3\":%d,\"b4\":%d,\"b5\":%d}",
             sensorBumpData.b0, sensorBumpData.b1, sensorBumpData.b2, sensorBumpData.b3, sensorBumpData.b4, sensorBumpData.b5);

    returnedArray[maxChars] = '\0';

}

//assumes that returnedArray is size maxChars + 1
void convertSensorIRToJSONString(struct sensorIRData_t sensorIRData,
                                   char *returnedArray, int maxChars)
{
    snprintf(returnedArray, maxChars, "{\"left\":%d,\"center\":%d,\"right\":%d}",
             sensorIRData.leftDistance, sensorIRData.centerDistance, sensorIRData.rightDistance);

    returnedArray[maxChars] = '\0';

}

//assumes that returnedArray is size maxChars + 1
void convertMotorToJSONString(struct motorData_t MotorData,
                                   char *returnedArray, int maxChars)
{
    snprintf(returnedArray, maxChars, "{\"rpmM1\":%d,\"rpmM2\":%d,\"dirM1\":%d,\"dirM2\":%d}",
             MotorData.rpmM1, MotorData.rpmM2, MotorData.dirM1, MotorData.dirM2);

    returnedArray[maxChars] = '\0';

}

//assumes that returnedArray is size maxChars + 1
char* convertControllerToJSONString(struct controllerData_t controllerData, int maxChars)
{
    char* returnedArray = malloc(maxChars * sizeof(char));
    snprintf(returnedArray, maxChars, "{\"pressed\":%d,\"key\":%s}",
             controllerData.pressed, controllerData.key);

    returnedArray[maxChars] = '\0';

    return returnedArray;

}

char createChecksum(char *payload)
{
    char ch = 0;
    int i;
    for (i = 0; i < strlen(payload); i++)
    {
        //XOR each bit
        ch = ch ^ payload[i];
    }
    if (ch == 0)
    {
        ch = 127;
    }
    return ch;
}

bool validateChecksum(char *payload)
{
    char ch = payload[CHECKSUM_LOC];

    char testCh = 0;
    int i;
    for (i = CHECKSUM_LOC + 1; i < strlen(payload); i++)
    {
        testCh = testCh ^ payload[i];
    }
    if (testCh == 0)
    {
        testCh = 127;
    }

    strncpy(payload, &payload[1], strlen(payload));
    return ch == testCh;
}
