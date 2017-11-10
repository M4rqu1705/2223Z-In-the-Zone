#include "drive.hpp"

namespace drive{

	void init(bool reset){
		counter = 0;

		invertButtonPressed = false;

		slewOutputs[0] = 0; slewOutputs[1] = 0;
		PIDoutput = 0;
		joystickInputs[0] = 0; joystickInputs[1] = 0;
		outputs[0] = 0; outputs[1] = 0;

		directionNormal = true;
		notDone = true;

		if(reset){
			encoderReset(drive::encoderL);	encoderReset(drive::encoderR);
			gyroReset(drive::driveGyro);
		}
		else{
			drive::encoderL = encoderInit(SENSOR_ENCODER_L, (SENSOR_ENCODER_L + 1), SENSOR_ENCODER_L_INVERTED);
			drive::encoderR = encoderInit(SENSOR_ENCODER_R, (SENSOR_ENCODER_R + 1), SENSOR_ENCODER_R_INVERTED);
		}


	}

	void operatorControl() {

		//Button toggle
		if (joystickGetDigital(1, JOYSTICK_DRIVE_INVERT, JOYSTICK_DRIVE_INVERT_BUTTON) == 1) {
			if (!invertButtonPressed) {
				invertButtonPressed = true;
				directionNormal = !directionNormal;
			}
		}
		else invertButtonPressed = false;

		//Assign joystick values to variables
		joystickInputs[0] = joystickGetAnalog(1, JOYSTICK_DRIVE_F);
		joystickInputs[1] = joystickGetAnalog(1, JOYSTICK_DRIVE_S);

		//Only use values if withing threshold. If not withing threshold, assign 0
		if (joystickInputs[0] < DRIVE_THRESHOLD && joystickInputs[0] > -DRIVE_THRESHOLD) joystickInputs[0] = 0;
		if (joystickInputs[1] < DRIVE_THRESHOLD && joystickInputs[1] > -DRIVE_THRESHOLD) joystickInputs[1] = 0;

		//Slewrate control. Assign value to output incrementing or decreasing values using SLEW_GAIN
		//If direction inverted, decrease values where normally increased and vice versa
		slewOutputs[0] += CLAMP(SLEW_GAIN * SIGN(joystickInputs[0], directionNormal? 1 : -1));
		slewOutputs[1] += CLAMP(SLEW_GAIN * SIGN(joystickInputs[1], directionNormal? 1 : -1));

		//Calculate "arcade drive" values for left and right side
		if (directionNormal) {
			outputs[0] = CLAMP(ROUND(slewOutputs[1]) + ROUND(slewOutputs[0]));
			outputs[1] = CLAMP(ROUND(slewOutputs[1]) - ROUND(slewOutputs[0]));
		}
		else {
			outputs[0] = CLAMP(ROUND(slewOutputs[1]) - ROUND(slewOutputs[0]));
			outputs[1] = CLAMP(ROUND(slewOutputs[1]) + ROUND(slewOutputs[0]));
		}

		//Move motors using calculated values for left and right side
		motorSet(MOTOR_DRIVE_LF, outputs[0]);
		motorSet(MOTOR_DRIVE_LB, outputs[0]);
		motorSet(MOTOR_DRIVE_RF, outputs[1]);
		motorSet(MOTOR_DRIVE_RB, outputs[1]);
	}

	void move(direction orientation, float pulses, int_fast8_t speed, bool useGyro) {
		//Recalculate pulses to convert them to degrees of rotation
		if (orientation == turnLeft || orientation == turnRight) {
			if (useGyro) pulses = DEGREES_ROTATION_TO_GYRO_TICKS(pulses);
			else pulses = DEGREES_ROTATION_TO_ENCODER_PULSES(pulses);
		}
		//Recaluclate pulses to convert them to inches of movement
		else if (orientation == forward || orientation == backward){
			 pulses = INCHES_TRANSLATION_TO_ENCODER_PULSES(pulses);
		 }

		//Calculate PID and rectify robot if necessary
		if (useGyro) {
			if (orientation == forward || orientation == backward) {    //Calculate PID using encoder values and rectify with gyro
				PIDoutput = drivePID.calculatePID(pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
				rectifyDriveGyro(outputs, PIDoutput, gyroGet(driveGyro));
			}
			else if (orientation == turnLeft || orientation == turnRight) {    //Calculate PID using gyro values and don't rectify
				PIDoutput = drivePID.calculatePID(pulses, gyroGet(driveGyro));
				outputs[0] = PIDoutput;
				outputs[1] = PIDoutput;
			}
		}
		else {
			//Calculate PID using encoder values
			PIDoutput = drivePID.calculatePID(pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
			//Rectify with encoders only if moving forward or backwards
			if (orientation == forward || orientation == backward) rectifyOutputsEncoder(outputs, PIDoutput, abs(encoderGet(encoderL)), abs(encoderGet(encoderR)));
			else {
				outputs[0] = PIDoutput;
				outputs[1] = PIDoutput;
			}
		}

		//PIDoutput = PID(PIDdrive, pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
		//outputs[0] = PIDoutput;
		//outputs[1] = PIDoutput;

		//Make sure left and right orientation values are within a range of values between -speed to speed
		outputs[0] = MAP(outputs[0], -127, 127, -speed, speed);
		outputs[1] = MAP(outputs[1], -127, 127, -speed, speed);

		//Move motors based on PID values, direction in which to move
		switch (orientation) {
		case forward:
		motorSet(MOTOR_DRIVE_LF, outputs[0]);
		motorSet(MOTOR_DRIVE_LB, outputs[0]);
		motorSet(MOTOR_DRIVE_RF, -outputs[1]);
		motorSet(MOTOR_DRIVE_RB, -outputs[1]);
			break;

		case backward:
		motorSet(MOTOR_DRIVE_LF, -outputs[0]);
		motorSet(MOTOR_DRIVE_LB, outputs[0]);
		motorSet(MOTOR_DRIVE_RF, outputs[1]);
		motorSet(MOTOR_DRIVE_RB, outputs[1]);
			break;

		case turnLeft:
		motorSet(MOTOR_DRIVE_LF, -outputs[0]);
		motorSet(MOTOR_DRIVE_LB, -outputs[0]);
		motorSet(MOTOR_DRIVE_RF, -outputs[1]);
		motorSet(MOTOR_DRIVE_RB, -outputs[1]);
			break;

		case turnRight:
		motorSet(MOTOR_DRIVE_LF, outputs[0]);
		motorSet(MOTOR_DRIVE_LB, outputs[0]);
		motorSet(MOTOR_DRIVE_RF, outputs[1]);
		motorSet(MOTOR_DRIVE_RB, outputs[1]);
			break;

		}

		//Check if drive is done
		if (PIDoutput < PID_DONE_THRESHOLD && PIDoutput > -PID_DONE_THRESHOLD) {
			if (counter < DRIVE_PID_CORRECTION_CYCLES) counter++;    //Sinchronous counter that doesn't affect other processes
			if (counter == DRIVE_PID_CORRECTION_CYCLES) {
				//If DRIVE_PID_CORRECTION_CYCLES time has passed since last time the robot was in position and it is still withing the threshold, it means that the drive was done
				PIDoutput = drivePID.calculatePID(pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
				if (useGyro && (orientation == turnLeft || orientation == turnRight)) {    //Calculate PID using gyro values and don't rectify
						PIDoutput = drivePID.calculatePID(pulses, gyroGet(driveGyro));
				}
				if (PIDoutput < PID_DONE_THRESHOLD && PIDoutput > -PID_DONE_THRESHOLD) notDone = false;
				else notDone = true;
				counter = 0;
			}
			else{
				notDone = true;
			}
		}
	}
}
