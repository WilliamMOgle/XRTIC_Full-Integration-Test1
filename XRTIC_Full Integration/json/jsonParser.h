
#ifndef JSONPARSER_H_
#define JSONPARSER_H_

#define CHECKSUM_LOC 0

#define ARM_PAYLOAD_SIZE 256

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>



struct simpleData_t
{
    int payloadID;
    int value;
    bool validData;
};

struct sevSegData_t {
    bool sA;
    bool sB;
    bool sC;
    bool sD;
    bool sE;
    bool sF;
    bool sG;
    bool sDP;

};

struct sensorRFIDData_t {
    char* type;
    char* effect;
};

struct sensorBumpData_t {
    bool b0;
    bool b1;
    bool b2;
    bool b3;
    bool b4;
    bool b5;
};

struct sensorIRData_t {
    int leftDistance;
    int centerDistance;
    int rightDistance;
};

struct motorData_t {
    int rpmM1;
    int rpmM2;

    bool dirM1;
    bool dirM2;
};

struct controllerData_t {
    bool pressed;
    char* key;
    bool newData;
    bool nfcEnabled;
};



struct simpleData_t parseSimpleJSON(char *jsonString, int maxTokens);

struct controllerData_t parseControllerJSON(char *jsonString, int maxTokens);

void convertSimpleDataToJSONString(struct simpleData_t simpleData,
                                   char *returnedArray, int maxChars);

void convertSevSegToJSONString(struct sevSegData_t sevSegData,
                                   char *returnedArray, int maxChars);

void convertSensorIRToJSONString(struct sensorIRData_t sensorIRData,
                                   char *returnedArray, int maxChars);

void convertSensorRFIDToJSONString(struct sensorRFIDData_t sensorRFIDData,
                                   char *returnedArray, int maxChars);

void convertSensorBumpToJSONString(struct sensorBumpData_t sensorBumpData,
                                   char *returnedArray, int maxChars);

void convertMotorToJSONString(struct motorData_t MotorData,
                                   char *returnedArray, int maxChars);

char* convertControllerToJSONString(struct controllerData_t controllerData, int maxChars);


char createChecksum(char *payload);
bool validateChecksum(char *payload);

#endif /* JSONPARSER_H_ */
