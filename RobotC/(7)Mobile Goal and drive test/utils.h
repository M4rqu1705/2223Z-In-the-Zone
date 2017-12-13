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

void calculatePID(robotArms &inStruct, signed int leftTarget, signed int rightTarget, signed int leftSensorInput, signed int rightSensorInput){
	//PID array format: KP(0), KI(1), KD(2), integralMax(3), error(4), lastError(5), integral(6), pidOutput(8)

	//Calculate error
	inStruct.PIDL[4] = (float)(leftTarget - leftSensorInput);

	//Calculate integral if within the integralLimit range
	if(fabs(inStruct.PIDL[6]+inStruct.PIDL[4]*(.001/LOOPS_DELAY))<=inStruct.PIDL[3]){
		inStruct.PIDL[6]+=(inStruct.PIDL[4]*(.001/LOOPS_DELAY));
	}
	if(inStruct.PIDL[4] == 0) inStruct.PIDL[6] = 0;


	inStruct.PIDL[8] =	inStruct.PIDL[0]*inStruct.PIDL[4] +
	inStruct.PIDL[1]*inStruct.PIDL[6] +
	inStruct.PIDL[2]*((inStruct.PIDL[4] - inStruct.PIDL[5])/LOOPS_DELAY*.001);

	inStruct.PIDL[5] = inStruct.PIDL[4];

	inStruct.PIDL[8] = CLAMP(inStruct.PIDL[8]);


	//Calculate error
	inStruct.PIDR[4] = (float)(rightTarget - rightSensorInput);

	//Calculate integral if within the integralLimit range
	if(fabs(inStruct.PIDR[6]+inStruct.PIDR[4]*(.001/LOOPS_DELAY))<=inStruct.PIDR[3]){
		inStruct.PIDR[6]+=(inStruct.PIDR[4]*(.001/LOOPS_DELAY));
	}
	if(inStruct.PIDR[4] == 0) inStruct.PIDR[6] = 0;


	inStruct.PIDR[8] =	inStruct.PIDR[0]*inStruct.PIDR[4] +
	inStruct.PIDR[1]*inStruct.PIDR[6] +
	inStruct.PIDR[2]*((inStruct.PIDR[4] - inStruct.PIDR[5])/LOOPS_DELAY*.001);

	inStruct.PIDR[5] = inStruct.PIDR[4];

	inStruct.PIDR[8] = CLAMP(inStruct.PIDR[8]);
}

#endif
