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
	if (inNumber >= 0) return ceil(inNumber - 0.49);	//Retreived from http://www.cplusplus.com/forum/beginner/3600/
	else return floor(inNumber + 0.49);
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


signed char calculatePID(float *values, signed int target, signed int current){
	//KP, KI, KD, integralMax, error, lastError, integral
	const signed char KP = 0, KI = 1, KD = 2, integralMax = 3, error = 4, lastError = 5, integral = 6;

	signed short PIDoutput = 0;

	values[error] = target - current;
	// values[integral] += (fabs(values[integral]) <= integralMax) ? values[error] : 0;
	values[integral] += values[error];
	if(values[error] == 0) values[integral] = 0;

	PIDoutput = values[KP]*values[error] + MAP((values[KI]*values[integral]), values[integralMax], -values[integralMax], 127, -127)	+ values[KD]*(values[error] - values[lastError]);
	// PIDoutput = values[KP]*values[error] + (values[KI]*values[integral])	+ values[KD]*(values[error] - values[lastError]);

	values[lastError] = values[error];

	return(CLAMP(PIDoutput));

}
