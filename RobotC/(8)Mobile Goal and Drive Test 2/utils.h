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

void calculatePID(float *values, signed int target, signed int sensorInput){
	//PID array format: KP(0), KI(1), KD(2), integralMax(3), error(4), lastError(5),
	//integral(6), correctionCycles(7), correctionThreshold(8), pidOutput(9)

	//Calculate error
	values[4] = (float)(target - sensorInput);

	//Calculate integral if within the integralLimit range
	if(fabs(values[6]+values[4]*(.001/LOOPS_DELAY))<=values[3]){
		values[6]+=(values[4]*(.001/LOOPS_DELAY));
	}
	if(values[4] == 0) values[6] = 0;


	values[9] =	values[0]*values[4] +
	values[1]*values[6] +
	values[2]*((values[4] - values[5])/LOOPS_DELAY*.001);

	values[5] = values[4];

	values[9] = CLAMP(values[9]);
}

void checkIfDone(robotDriveStruct &values){
	if(values.PID[9] <= values.PID[8] && values.PID[9] >= -values.PID[8]){
		if(values.counter<values.PID[7]) values.counter++;
		if(values.counter == values.PID[7]){
			if(values.PID[9] <= values.PID[8] && values.PID[9] >= -values.PID[8])	values.notDone = false;
			else values.notDone = true;
		}
	}
	else values.counter = 0;
}

void checkIfDone(robotMobileGoalIntake &values){
	if(values.PID[9] <= values.PID[8] && values.PID[9] >= -values.PID[8]){
		if(values.counter < values.PID[7]) values.counter++;
		if(values.counter == values.PID[7]){
			if(values.PID[9] <= values.PID[8] && values.PID[9] >= -values.PID[8])	values.notDone = false;
			else values.notDone = true;
		}
	}
	else values.counter = 0;
}

#endif
