/*
 * main_helper.c
 *
 *  Created on: Apr 30, 2021
 *      Author: wills
 */

#include "main.h"

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
