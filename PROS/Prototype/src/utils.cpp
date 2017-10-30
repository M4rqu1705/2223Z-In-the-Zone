#include <utils.hpp>

int_fast32_t ROUND(float inNumber) {
	if (inNumber >= 0) return ceil(inNumber - 0.49);	//Retreived from http://www.cplusplus.com/forum/beginner/3600/
	else return ceil(inNumber + 0.49);
}

int_fast16_t MAP(int_fast16_t inNumber, int_fast16_t inMin, int_fast16_t inMax, int_fast16_t outMin, int_fast16_t outMax) {
	return ((inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}
//Retreived from https://www.arduino.cc/en/Reference/Map

int_fast8_t CLAMP(int_fast16_t inNumber) {
	if (inNumber > 127) return 127;
	else if (inNumber < -127) return -127;
	return inNumber;
}

uint_fast16_t INCHES_TRANSLATION_TO_ENCODER_PULSES(float inches) {
	return ROUND(inches*(360 / (WHEEL_DIAMETER*PI)));
}

int_fast16_t DEGREES_ROTATION_TO_GYRO_TICKS(float targetDegrees) {
	return ROUND(targetDegrees *(GYRO_FULL_ROTATION_TICKS / 360));
}

int_fast16_t DEGREES_ROTATION_TO_ENCODER_PULSES(float targetDegrees) {
	return ROUND((targetDegrees*DRIVE_WIDTH) / WHEEL_DIAMETER);
}
//Retreived from https://en.wikipedia.org/wiki/Arc_(geometry)#Length_of_an_arc_of_a_circle

void rectifyOutputsEncoder(int_fast16_t *values, int_fast8_t speed, int_fast16_t leftSideSensor, int_fast16_t rightSideSensor) {
	values[0] = speed - (leftSideSensor - rightSideSensor)*RECTIFY_CONSTANT_ENCODER;
	values[1] = speed - (rightSideSensor - leftSideSensor)*RECTIFY_CONSTANT_ENCODER;
}

void rectifyDriveGyro(int_fast16_t *values, int_fast8_t speed, int_fast16_t gyroSensor) {
	values[0] = speed - ((gyroSensor)*RECTIFY_CONSTANT_GYRO);
	values[1] = speed + ((gyroSensor)*RECTIFY_CONSTANT_GYRO);
}
