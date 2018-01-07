#ifndef fullRobotTestUtils.h
#define fullRobotTestUtils.h

#pragma systemfile

int MAP(int inNumber, int inMax, int inMin, int outMax, int outMin){
	return ((inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
	//Retrieved from https://www.arduino.cc/reference/en/language/functions/math/map/
}

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


unsigned short INCHEStoPULSES(float inches) {
	return ROUND(inches*(360/(META_driveWheelDiameter*PI)));
}

signed short DEGREEStoPULSES(float targetDegrees) {
	return ROUND((targetDegrees*META_driveWidth)/META_driveWheelDiameter);
}
//Retreived from https://en.wikipedia.org/wiki/Arc_(geometry)#Length_of_an_arc_of_a_circle

void rectifyOutputsEncoder(signed byte *values, signed byte speed, signed int leftSideSensor, signed int rightSideSensor) {
	values[0] = speed - (leftSideSensor - rightSideSensor)*META_driveEncoderRectifyConstant;
	values[1] = speed - (rightSideSensor - leftSideSensor)*META_driveEncoderRectifyConstant;
}

//PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID//

void calculatePID(float *values, signed int target, signed int sensorInput){
	//PID array format: KP(0), KI(1), KD(2), integralMax(3), error(4), lastError(5),
	//integral(6), correctionCycles(7), correctionThreshold(8), pidOutput(9)

	//Calculate error
	values[4] = (float)(target - sensorInput);

	//Calculate integral if within the integralLimit range
	if(fabs(values[6]+(values[4]*0.001*LOOPS_DELAY))<=values[3]){
		values[6]+=(values[4]*0.001*LOOPS_DELAY);
	}
	if(values[4] <= 0 + values[8] && values[4] >= 0 - values[8]) values[6] = 0;

	/*writeDebugStream("Values = ");
	for(int C = 0; C<10; C++){
		writeDebugStream("%f, ", values[C]);
	}
	writeDebugStreamLine("");
	datalogAddValue(0, sensorInput);*/

	values[9] =	values[0]*values[4] + values[1]*values[6] + values[2]*((values[4] - values[5])/(LOOPS_DELAY*0.05));
	values[5] = values[4];
	values[9] = CLAMP(values[9]);
}

void checkIfDriveDone(robotDriveStruct &values){
	if(values.PID[9] <= values.PID[8] && values.PID[9] >= -values.PID[8]){
		if(values.counter<(unsigned byte)values.PID[7]) values.counter++;
		if(values.counter == (unsigned byte)values.PID[7]){
			if(values.PID[9] <= values.PID[8] && values.PID[9] >= -values.PID[8])	values.notDone = false;
			else values.notDone = true;
		}
	}
	else values.counter = 0;
}

void checkIfMogoDone(robotMobileGoalIntakeStruct &values){
	if(values.PID[9] <= values.PID[8] && values.PID[9] >= -values.PID[8]){
		if(values.counter < (unsigned byte)values.PID[7]) values.counter++;
		if(values.counter == (unsigned byte)values.PID[7]){
			if(values.PID[9] <= values.PID[8] && values.PID[9] >= -values.PID[8])	values.notDone = false;
			else values.notDone = true;
		}
	}
	else values.counter = 0;
}

void checkIfArmDone(robotArmStruct &values){
	if(values.PID[9] <= values.PID[8] && values.PID[9] >= -values.PID[8]){
		if(values.counter < (unsigned byte)values.PID[7]) values.counter++;
		if(values.counter == (unsigned byte)values.PID[7]){
			if(values.PID[9] <= values.PID[8] && values.PID[9] >= -values.PID[8])	values.notDone = false;
			else values.notDone = true;
		}
	}
	else values.counter = 0;
}

#endif
