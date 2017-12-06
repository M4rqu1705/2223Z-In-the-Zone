#include "utils.h"

signed int MAP(signed int inNumber, signed int inMax, signed int inMin, signed int outMax, signed int outMin){
	return ((inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}

signed char CLAMP(signed int inNumber){
	if(inNumber>127) return 127;
	else if (inNumber<-127) return -127;
	else return inNumber;
}

signed int ROUND(float inNumber) {
	if (inNumber >= 0) return (signed int)(ceil(inNumber - 0.49));	//Retreived from http://www.cplusplus.com/forum/beginner/3600/
	else return (signed int)(floor(inNumber + 0.49));
}

void toggleButton(unsigned char buttonGroup, unsigned char button, bool &buttonToggleVariable, returnFunction function){
	if(joystickGetDigital(1, buttonGroup, button) == 1){
		if(!buttonToggleVariable){
			buttonToggleVariable = true;
			function();
		}
	}
	else buttonToggleVariable = false;
}

namespace PID{
	signed char calculatePID(float *values, signed int target, signed int current){
		//KP, KI, KD, integralMax, error, lastError, integral
		const signed char KP = 0, KI = 1, KD = 2, integralMax = 3, error = 4, lastError = 5, integral = 6;

		signed short PIDoutput = 0;

		values[error] = target - current;

		if(fabs(values[integral]+values[error]*(.01/AUTON_LOOP_DELAY))<=values[integralMax]){
			values[integral]+=(values[error]*(.01/AUTON_LOOP_DELAY));
		}
		if(values[error] == 0) values[integral] = 0;

		PIDoutput = values[KP]*values[error] + values[KI]*values[integral] + values[KD]*((values[error] - values[lastError])/AUTON_LOOP_DELAY*.01);

		values[lastError] = values[error];

		return(CLAMP(PIDoutput));

	}
}
