/*   utils.h - Useful mathematical functions                                *
*    Copyright (C) <2017>  Marcos Ricardo Pesante Colón                     *
*                                                                           *
*    This program is free software: you can redistribute it and/or modify   *
*    it under the terms of the GNU General Public License as published by   *
*    the Free Software Foundation, either version 3 of the License, or      *
*    (at your option) any later version.                                    *
*                                                                           *
*    This program is distributed in the hope that it will be useful,        *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of         *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
*    GNU General Public License for more details.                           *
*                                                                           *
*    You should have received a copy of the GNU General Public License      *
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#ifndef utils.h
#define utils.h

#pragma systemfile

//Conversions
signed int MAP(signed int inNumber, signed int inMax, signed int inMin, signed int outMax, signed int outMin){
	return ((inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}
//Retrieved from https://www.arduino.cc/reference/en/language/functions/math/map/

signed byte CLAMP(signed int inNumber){
	if(inNumber>127) return 127;
	else if (inNumber<-127) return -127;
	else return inNumber;
}

signed int ROUND(float inNumber) {
	if (inNumber >= 0) return (signed int)(ceil(inNumber - 0.49));
	else return (signed int)(floor(inNumber + 0.49));
	//Retreived from http://www.cplusplus.com/forum/beginner/3600/
}


unsigned short INCHES_TRANSLATION_TO_ENCODER_PULSES(float inches) {
	return ROUND(inches*(360/(WHEEL_DIAMETER*PI)));
}

signed short DEGREES_ROTATION_TO_ENCODER_PULSES(float targetDegrees) {
	return ROUND((targetDegrees*DRIVE_WIDTH)/WHEEL_DIAMETER);
}
//Retreived from https://en.wikipedia.org/wiki/Arc_(geometry)#Length_of_an_arc_of_a_circle

void rectifyOutputsEncoder(signed byte *values, signed byte speed, signed int leftSideSensor, signed int rightSideSensor) {
	values[0] = speed - (leftSideSensor - rightSideSensor)*RECTIFY_CONSTANT_ENCODER;
	values[1] = speed - (rightSideSensor - leftSideSensor)*RECTIFY_CONSTANT_ENCODER;
}

//PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID//

void calculatePID(driveStruct &inStruct, signed int target, signed int sensorInput){
	//PID array format: KP(0), KI(1), KD(2), integralMax(3), error(4), lastError(5), integral(6), pidOutput(8)

	//Calculate error
	inStruct.PID[4] = (float)(target - sensorInput);

	//Calculate integral if within the integralLimit range
	if(fabs(inStruct.PID[6]+inStruct.PID[4]*(.001/LOOPS_DELAY))<=inStruct.PID[3]){
		inStruct.PID[6]+=(inStruct.PID[4]*(.001/LOOPS_DELAY));
	}
	if(inStruct.PID[4] == 0) inStruct.PID[6] = 0;


	inStruct.PID[8] =	inStruct.PID[0]*inStruct.PID[4] +
	inStruct.PID[1]*inStruct.PID[6] +
	inStruct.PID[2]*((inStruct.PID[4] - inStruct.PID[5])/LOOPS_DELAY*.001);

	inStruct.PID[5] = inStruct.PID[4];

	inStruct.PID[8] = CLAMP(inStruct.PID[8]);
}

void calculatePID(robotMobileGoalIntake &inStruct, signed int target, signed int sensorInput){
	//PID array format: KP(0), KI(1), KD(2), integralMax(3), error(4), lastError(5), integral(6), pidOutput(8)

	//Calculate error
	inStruct.PID[4] = (float)(target - sensorInput);

	//Calculate integral if within the integralLimit range
	if(fabs(inStruct.PID[6]+inStruct.PID[4]*(.001/LOOPS_DELAY))<=inStruct.PID[3]){
		inStruct.PID[6]+=(inStruct.PID[4]*(.001/LOOPS_DELAY));
	}
	if(inStruct.PID[4] == 0) inStruct.PID[6] = 0;


	inStruct.PID[8] =	inStruct.PID[0]*inStruct.PID[4] +
	inStruct.PID[1]*inStruct.PID[6] +
	inStruct.PID[2]*((inStruct.PID[4] - inStruct.PID[5])/LOOPS_DELAY*.001);

	inStruct.PID[5] = inStruct.PID[4];

	inStruct.PID[8] = CLAMP(inStruct.PID[8]);
}

#endif
