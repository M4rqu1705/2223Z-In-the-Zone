#include "main.h"

const signed char JOYS_DRIVE_F = 3;
const signed char JOYS_DRIVE_S = 1;

const signed char MOTOR_DRIVE_LB = 1;
const signed char MOTOR_DRIVE_LF = 2;
const signed char MOTOR_DRIVE_RF = 9;
const signed char MOTOR_DRIVE_RB = 10;

const signed char JOYS_MOGO_GROUP = 6;
const signed char JOYS_MOGO_BUTTON = JOY_DOWN;

const signed char MOTOR_MOGO_L = 3;
const signed char MOTOR_MOGO_R = 8;

const signed char MOGO_MOTOR_SPEED = 127;
signed char MOGO_ACTIVE_MOTOR_TIME = 100;

short driveOutputs[2];

bool toggleMogoButton = false;
bool retract = false;

unsigned char mogoCounter = 25;

signed char mogoOutputs[2] = {0};


void operatorControl() {
	while (true) {


		if(joystickGetDigital(1, JOYS_MOGO_GROUP, JOYS_MOGO_BUTTON) == 1){
			if(!toggleMogoButton){
				toggleMogoButton = true;
				retract = !retract;
				mogoCounter = 0;
			}
		}
		else toggleMogoButton = false;

		if(retract){
				mogoOutputs[0] = MOGO_MOTOR_SPEED;
				mogoOutputs[1] = -MOGO_MOTOR_SPEED;
				MOGO_ACTIVE_MOTOR_TIME = 50;
		}
		else{
				mogoOutputs[0] = -MOGO_MOTOR_SPEED;
				mogoOutputs[1] = MOGO_MOTOR_SPEED;
				MOGO_ACTIVE_MOTOR_TIME = 100;
		}

		if(mogoCounter < MOGO_ACTIVE_MOTOR_TIME){
			motorSet(MOTOR_MOGO_L, mogoOutputs[0]);
			motorSet(MOTOR_MOGO_R, mogoOutputs[1]);
		}
		else{
			motorSet(MOTOR_MOGO_L, 0);
			motorSet(MOTOR_MOGO_R, 0);
		}

		driveOutputs[0] = joystickGetAnalog(1, JOYS_DRIVE_S) + joystickGetAnalog(1, JOYS_DRIVE_F);	//Store the drive�s left side values in driveOutputs[0]
		driveOutputs[1] = joystickGetAnalog(1, JOYS_DRIVE_S) - joystickGetAnalog(1, JOYS_DRIVE_F);	//Store the drive�s right side values in driveOutputs[1]

		//Set motor speeds based on calculated values
		motorSet(MOTOR_DRIVE_LB, -driveOutputs[0]);
		motorSet(MOTOR_DRIVE_LF, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_RF, driveOutputs[1]);
		motorSet(MOTOR_DRIVE_RB, -driveOutputs[1]);

		if(mogoCounter<=MOGO_ACTIVE_MOTOR_TIME) mogoCounter++;

		delay(20);	//Delay 20 milliseconds so the processor doesn't overheat, but still can update the motors at a constant rate
	}
}
