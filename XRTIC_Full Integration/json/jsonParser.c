/*
 * jsonParser.c
 *
 *  Created on: Sep 21, 2020
 *      Author: onemo
 */

#include "jsmn.h"
#include "jsonParser.h"

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
    free(keyString);
    free(keyString2);

    return cD;
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
    snprintf(returnedArray, maxChars, "{\"detect\":%d,\"type\":\"%s\",\"effect\":\"%s\"}",
             sensorRFIDData.detect, sensorRFIDData.type, sensorRFIDData.effect);

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
void convertMotorToJSONString(struct motorData_t motorData,
                                   char *returnedArray, int maxChars)
{
    snprintf(returnedArray, maxChars, "{\"rpmM1\":%d,\"rpmM2\":%d,\"dirM1\":%d,\"dirM2\":%d}",
             motorData.rpmM1, motorData.rpmM2, motorData.dirM1, motorData.dirM2);

    returnedArray[maxChars] = '\0';

}

//assumes that returnedArray is size maxChars + 1
void convertIRSensorToJSONString(struct sensorIRData_t irData,
                                   char *returnedArray, int maxChars)
{
    snprintf(returnedArray, maxChars, "{\"left\":%d,\"center\":%d,\"right\":%d}",
             irData.left, irData.center, irData.right);

    returnedArray[maxChars] = '\0';

}
