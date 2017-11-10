#ifndef DRIVE_H_INCLUDED_
#define DRIVE_H_INCLUDED_

#include "main.h"
	#include "PID.h"

namespace drive {
	uint_fast8_t counter;
	bool invertButtonPressed;

	PID drivePID(DRIVE_PID_KP_PRESET, DRIVE_PID_KI_PRESET, DRIVE_PID_KD_PRESET, DRIVE_PID_INTEGRAL_LIMIT_PRESET);

	float slewOutputs[2];  //Declare Slewrate variables to store SLEWCHANGE
	int_fast16_t PIDoutput;  //Declare shorts to store the outputs of the PID calculation and the individual drive sides output after rectifying the PID output
	int_fast8_t joystickInputs[2], outputs[2];      //Declare array to store joystick values (0 = powerOutput, 1 = turnOutput) so it is not necessary to retrieve the value more than once (efficiency purposes)
	bool directionNormal, notDone;     //Declare booleans to indicate direction state during driver control and if the drive is done moving during autonomous
	enum direction : uint_fast8_t { forward, backward, turnLeft, turnRight };        //Create enumerated type variable to indicate direciton

	Gyro driveGyro;
	Encoder encoderL;
	Encoder encoderR;

	void init(bool reset = false);

	void operatorControl();
	void move(direction orientation, float pulses, int_fast8_t speed, bool useGyro);
}

#endif
