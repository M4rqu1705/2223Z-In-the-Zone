#include "main.h"

int_fast16_t map(int_fast16_t inNumber, int_fast16_t inMin, int_fast16_t inMax, int_fast16_t outMin, int_fast16_t outMax) {
	return (inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

int_fast8_t withinRange(int_fast16_t inNumber) {
	return abs(inNumber) > 127 ? inNumber < -127 ? -127 : 127 : 0;
}

int_fast16_t inchesOfTranslationToEncoderPulses(int_fast16_t inches) {
	return (WHEEL_DIAMETER*3.1415926535897932384626433832795)*(inches / 360);
}

int_fast16_t degreesOfRotationToGyroTicks(int_fast16_t degrees) {
	return ceil(degrees *(GYRO_FULL_ROTATION_TICKS / 360));
}

int_fast16_t degreesOfRotationToEncoderPulses(int_fast16_t degrees) {
	return floor(((degrees / 360) * DRIVE_WIDTH * 3.1415926535897932384626433832795) / (WHEEL_DIAMETER / 360));
}

void rectifyOutputs(int_fast16_t *values, int_fast16_t speed, uint_fast16_t leftSideSensor, uint_fast16_t rightSideSensor) {
	values[0] = (speed / (leftSideSensor - rightSideSensor)) * 0.01 + 1;
	values[1] = (speed / (rightSideSensor - leftSideSensor)) * 0.01 + 1;
}
