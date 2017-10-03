#include "main.h"

signed short map(signed short inNumber, signed short inMin, signed short inMax, signed short outMin, signed short outMax) {
	return (inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

signed char withinRange(signed short inNumber) {
	return abs(inNumber) > 127 ? inNumber < -127 ? -127 : 127 : 0;
}

signed short inchesOfTranslationToEncoderPulses(signed short inches) {
	return (WHEEL_DIAMETER*3.1415926535897932384626433832795)*(inches / 360);
}

signed short degreesOfRotationToGyroTicks(signed short degrees) {
	return ceil(degrees *(GYRO_FULL_ROTATION_TICKS / 360));
}

signed short degreesOfRotationToEncoderPulses(signed short degrees) {
	return floor(((degrees / 360) * DRIVE_WIDTH * 3.1415926535897932384626433832795) / (WHEEL_DIAMETER / 360));
}

void rectifyOutputs(signed short *values, signed short speed, int leftSideSensor, int rightSideSensor) {
	values[0] = (speed / (leftSideSensor - rightSideSensor)) * 0.01 + 1;
	values[1] = (speed / (rightSideSensor - leftSideSensor)) * 0.01 + 1;
}
