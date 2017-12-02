#include "utils.h"

signed int ROUND(float inNumber) {
	if (inNumber >= 0) return ceil(inNumber - 0.49);	//Retreived from http://www.cplusplus.com/forum/beginner/3600/
	else return floor(inNumber + 0.49);
}

signed short INCHES_TRANSLATION_TO_ENCODER_PULSES(float inches) {
	return ROUND(inches*(360 / (WHEEL_DIAMETER*PI)));
}

signed short DEGREES_ROTATION_TO_ENCODER_PULSES(float targetDegrees) {
	return ROUND((targetDegrees*DRIVE_WIDTH) / WHEEL_DIAMETER);
}
//Retreived from https://en.wikipedia.org/wiki/Arc_(geometry)#Length_of_an_arc_of_a_circle

signed short DEGREES_ROTATION_TO_GYRO_TICKS(float targetDegrees) {
	return ROUND(targetDegrees *(GYRO_FULL_ROTATION_TICKS / 360));
}

void rectifyOutputsEncoder(signed char *values, int_fast8_t speed, signed int leftSideSensor, signed int rightSideSensor) {
	values[0] = speed - (leftSideSensor - rightSideSensor)*RECTIFY_CONSTANT_ENCODER;
	values[1] = speed - (rightSideSensor - leftSideSensor)*RECTIFY_CONSTANT_ENCODER;
}

void rectifyDriveGyro(signed char *values, int_fast8_t speed, signed int gyroSensor) {
	values[0] = speed - ((gyroSensor)*RECTIFY_CONSTANT_GYRO);
	values[1] = speed + ((gyroSensor)*RECTIFY_CONSTANT_GYRO);
}


namespace PID {

	void reset(){
		output = 0;

		PIDdrive[0] = DRIVE_PID_KP_PRESET;
		PIDdrive[1] = DRIVE_PID_KI_PRESET;
		PIDdrive[2] = DRIVE_PID_KD_PRESET;
		PIDdrive[3] = 0;
		PIDdrive[4] = 0;
		PIDdrive[5] = DRIVE_PID_INTEGRAL_LIMIT_PRESET;
		PIDdrive[6] = 0;

		PIDarmL[0] = ARM_PID_KP_PRESET;
		PIDarmL[1] = ARM_PID_KI_PRESET;
		PIDarmL[2] = ARM_PID_KD_PRESET;
		PIDarmL[3] = 0;
		PIDarmL[4] = 0;
		PIDarmL[5] = ARM_PID_INTEGRAL_LIMIT_PRESET;
		PIDarmL[6] = 0;

		PIDarmR[0] = ARM_PID_KP_PRESET;
		PIDarmR[1] = ARM_PID_KI_PRESET;
		PIDarmR[2] = ARM_PID_KD_PRESET;
		PIDarmR[3] = 0;
		PIDarmR[4] = 0;
		PIDarmR[5] = ARM_PID_INTEGRAL_LIMIT_PRESET;
		PIDarmR[6] = 0;
	}

	signed char calculatePID(float *values, signed short target, signed int sensorInput){
	//values[] array format: KP(0),  KI(1), KD(2), error(3), integral(4), integralLimit(5), lastError(6)

		//Calculate error
		values[3] = float(target-sensorInput);

		//Calculate integral if within the integralLimit range
		values[4]+=(abs(int(values[4])) + abs(int(values[3])) < values[5]) ? values[3] : 0;
		if(values[3] == 0) values[4] = 0;

		//Calculate output value
		output = values[0]*values[3] + MAP((values[1]*values[4]), -values[5], values[5], -127, 127) + values[2]*(values[3]-values[6]);

		values[6] = values[3];

		return ((signed char) (CLAMP(output)));
	}
}
