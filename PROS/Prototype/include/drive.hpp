#ifndef DRIVE_HPP_
#define DRIVE_HPP_

#include <API.hpp>
#include <utils.hpp>

namespace drive {
	counterType counter;
	toggleButton invertButtonPressed;

	float powerOutput, turnOutput;  //Declare Slewrate variables to store SLEWCHANGE
	int_fast16_t PIDoutput;  //Declare shorts to store the outputs of the PID calculation and the individual drive sides output after rectifying the PID output
	int_fast8_t joystickInputs[2], outputs[2];      //Declare array to store joystick values (0 = powerOutput, 1 = turnOutput) so it is not necessary to retrieve the value more than once (efficiency purposes)
	bool directionNormal, notDone;     //Declare booleans to indicate direction state during driver control and if the drive is done moving during autonomous
	enum direction : uint_fast8_t { forward = 0, backward, turnLeft, turnRight };        //Create enumerated type variable to indicate direciton

	Gyro driveGyro;
	Encoder encoderL;
	Encoder encoderR;

	void operatorControl();
	void move(direction orientation, float pulses, int_fast8_t speed, bool useGyro = false);

}

#endif
