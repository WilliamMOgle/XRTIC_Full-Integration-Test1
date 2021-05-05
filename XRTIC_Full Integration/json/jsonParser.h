
#ifndef JSONPARSER_H_
#define JSONPARSER_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


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
    bool detect;
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
    int left;
    int center;
    int right;
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



struct controllerData_t parseControllerJSON(char *jsonString, int maxTokens);


void convertSevSegToJSONString(struct sevSegData_t sevSegData,
                                   char *returnedArray, int maxChars);

void convertSensorRFIDToJSONString(struct sensorRFIDData_t sensorRFIDData,
                                   char *returnedArray, int maxChars);

void convertSensorBumpToJSONString(struct sensorBumpData_t sensorBumpData,
                                   char *returnedArray, int maxChars);

void convertMotorToJSONString(struct motorData_t MotorData,
                                   char *returnedArray, int maxChars);

void convertIRSensorToJSONString(struct sensorIRData_t MotorData,
                                   char *returnedArray, int maxChars);



#endif /* JSONPARSER_H_ */
