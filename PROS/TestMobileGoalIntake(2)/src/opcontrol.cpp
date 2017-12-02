#include "main.h"


namespace drive{
	short motorOutputs[2] = {0};
	bool toggleDirectionButton = false, normalDirection = true;

	void normalEqualsNotDriveNormal(){
		normalDirection = !normalDirection;
	}
}

namespace mobileGoal{
	signed char motorOutputs[2] = {0};

	float PID[7] = {3 ,1 ,3, 1000, 0, 0, 0 };

	bool toggleRetractButton = false;
	bool toggleExtendButton = false;
	bool retract = false;

	void retractEqualsFalse(){
		retract = false;
		PID[0] = 3;
		PID[1] = 1;
		PID[2] = 3;
		PID[3] = 1000;
		PID[4] = 0;
		PID[5] = 0;
		PID[6] = 0;
	}

	void retractEqualsTrue(){
		retract = true;
		PID[0] = 3;
		PID[1] = 1;
		PID[2] = 3;
		PID[3] = 1000;
		PID[4] = 0;
		PID[5] = 0;
		PID[6] = 0;
	}
}


void operatorControl() {
	while (true) {

		toggleButton(JOYS_DRIVE_INVERT_GROUP, JOYS_DRIVE_INVERT_BUTTON, drive::toggleDirectionButton, drive::normalEqualsNotDriveNormal);

		toggleButton(JOYS_MOGO_EXTEND_GROUP, JOYS_MOGO_EXTEND_BUTTON, mobileGoal::toggleExtendButton, mobileGoal::retractEqualsFalse);

		toggleButton(JOYS_MOGO_RETRACT_GROUP, JOYS_MOGO_RETRACT_BUTTON, mobileGoal::toggleRetractButton, mobileGoal::retractEqualsTrue);


		if(drive::normalDirection){
			drive::motorOutputs[0] = joystickGetAnalog(1, JOYS_DRIVE_S) + joystickGetAnalog(1, JOYS_DRIVE_F);	//Store the drive's left side values in drive::motorOutputs[0]
			drive::motorOutputs[1] = joystickGetAnalog(1, JOYS_DRIVE_S) - joystickGetAnalog(1, JOYS_DRIVE_F);	//Store the drive's right side values in drive::motorOutputs[1]
		}
		else{
			drive::motorOutputs[0] = joystickGetAnalog(1, JOYS_DRIVE_S) - joystickGetAnalog(1, JOYS_DRIVE_F);	//Store the drive's left side values in drive::motorOutputs[0]
			drive::motorOutputs[1] = joystickGetAnalog(1, JOYS_DRIVE_S) + joystickGetAnalog(1, JOYS_DRIVE_F);	//Store the drive's right side values in drive::motorOutputs[1]
		}


		if(mobileGoal::retract){
			mobileGoal::motorOutputs[0] = calculatePID(mobileGoal::PID, MOGO_POT_RETRACTED, analogRead(SENSOR_MOGO_POT));
			mobileGoal::motorOutputs[1] = -calculatePID(mobileGoal::PID, MOGO_POT_RETRACTED, analogRead(SENSOR_MOGO_POT));
		}
		else{
			mobileGoal::motorOutputs[0] = calculatePID(mobileGoal::PID, MOGO_POT_EXTENDED, analogRead(SENSOR_MOGO_POT));
			mobileGoal::motorOutputs[1] = -calculatePID(mobileGoal::PID, MOGO_POT_EXTENDED, analogRead(SENSOR_MOGO_POT));
		}

		motorSet(MOTOR_MOGO_L, mobileGoal::motorOutputs[0]);
		motorSet(MOTOR_MOGO_R, mobileGoal::motorOutputs[1]);

		for(int c = 0; c<7; c++){
			printf("%f, ", mobileGoal::PID[c]);
		}
		printf("\n");

		//Set motor speeds based on calculated values
			motorSet(MOTOR_DRIVE_LB, -drive::motorOutputs[0]);
			motorSet(MOTOR_DRIVE_LF, drive::motorOutputs[0]);
			motorSet(MOTOR_DRIVE_RF, drive::motorOutputs[1]);
			motorSet(MOTOR_DRIVE_RB, -drive::motorOutputs[1]);


		delay(20);	//Delay 20 milliseconds so the processor doesn't overheat, but still can update the motors at a constant rate
	}
}