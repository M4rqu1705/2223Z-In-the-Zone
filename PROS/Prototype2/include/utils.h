#ifndef UTILS_H_INCLUDED_
#define UTILS_H_INCLUDED_

#include "API.h"
#include "commons.h"

#define MAP(inNumber, inMax, inMin, outMax, outMin) (((inNumber) - (inMin)) * ((outMax) - (outMin)) / ((inMax) - (inMin)) + (outMin))
//Retreived from https://www.arduino.cc/en/Reference/Map

#define CLAMP(inNumber) (signed char)((abs(int(inNumber))>127) ? (int(inNumber)<-127) ? -127 : 127 : (int(inNumber)))

#define SIGN(inNumber, multiplier) (signed int)((int(inNumber)<0) ? -1*abs(int(multiplier)) : (int(inNumber)>0) ? abs(int(multiplier)) : 0)

#define WITHIN_THRESHOLD(limitMin, inNumber, limitMax) (int(inNumber)<=int(limitMax) && int(inNumber)>= int(limitMin))



signed int ROUND(float inNumber);
signed short INCHES_TRANSLATION_TO_ENCODER_PULSES(float inches);
signed short DEGREES_ROTATION_TO_ENCODER_PULSES(float targetDegrees);
signed short DEGREES_ROTATION_TO_GYRO_TICKS(float targetDegrees);

void rectifyOutputsEncoder(signed char *values, signed char speed, signed int leftSideSensor, signed int rightSideSensor);
void rectifyDriveGyro(signed char *values, signed char speed, signed int gyroSensor);


namespace PID {
	float PIDdrive[7], PIDarmL[7], PIDarmR[7];    //Declare PID arrays for drive and arms

	signed short output;

	void reset();
	signed char calculatePID(float *values, signed short target, signed int sensorInput);
}

#endif
