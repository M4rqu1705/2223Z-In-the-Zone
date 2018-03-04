#ifndef FAST_H_
#define FAST_H_

void FAST_pickup(ENUM_driveMode mode, float firstDistance, float firstSpeed, int firstTimeout, float secondDistance, float secondSpeed, int secondTimeout, bool driveRectify, bool resetGyro, bool coneIntakeRetract){

	resetValues();
	LOADED_mobileGoal(false, false, false, false);
	LOADED_arm(true);
	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	drive.motionProfile.distanceMultiplier[0] = 0.25;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = (int)(40);
	drive.motionProfile.offsets[1] = (int)(firstSpeed);
	drive.rectify = driveRectify;
	writeDebugStream("Before First loop\n\n");
	for(int C = 0; drive.PID.notDone && C<(int)(firstTimeout/3); C++){
		if(firstDistance >= 0) DRIVE_forward(MtnPrfl, (float)(firstDistance/3), firstSpeed);
		else if(firstDistance < 0) DRIVE_backwards(MtnPrfl, (fabs(firstDistance)/3), firstSpeed);
		if(coneIntakeRetract) GOLIATH_pickUp(true, 100);
		MOBILEGOAL_retract(false);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
		writeDebugStream("FirstLoop\n\n");
	}

	resetValues();
	drive.rectify = driveRectify;
	LOADED_mobileGoal(false, false, false, false);
	for(int C = 0; drive.PID.notDone && C<firstTimeout; C++){
		if(firstDistance > 0) DRIVE_forward(PID, firstDistance, firstSpeed);
		else if(firstDistance < 0) DRIVE_backwards(PID, fabs(firstDistance), firstSpeed);
		if(coneIntakeRetract) GOLIATH_pickUp(true, 100);
		MOBILEGOAL_retract(false);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
		writeDebugStream("SecondLoop\n\n");
	}
	//Begin retracting the Mobile Goal Intake for 100 milliseconds before driving back
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	for(int C = 0; C<=10; C++){
		MOBILEGOAL_retract(true);
		if(coneIntakeRetract) GOLIATH_pickUp(true, 100);
		delay(META_loopsDelay);
	}

	if(resetGyro) SensorValue[SENSOR_gyro] = 0;    delay(META_loopsDelay);

	resetValues();
	LOADED_mobileGoal(true, true, false, false);
	LOADED_arm(true);
	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	drive.motionProfile.distanceMultiplier[0] = 0.25;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = 40;
	drive.motionProfile.offsets[1] = (int)(secondSpeed);
	drive.rectify = driveRectify;
	for(int C = 0; drive.PID.notDone && C<(int)(secondTimeout/4); C++){
		if(secondDistance > 0) DRIVE_forward(MtnPrfl, secondDistance/5, secondSpeed);
		else if(secondDistance < 0) DRIVE_backwards(MtnPrfl, fabs(secondDistance)/5, secondSpeed);
		if(coneIntakeRetract) GOLIATH_pickUp(true, 100);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	drive.rectify = driveRectify;
	for(int C = 0; drive.PID.notDone && C<secondTimeout; C++){
		if(secondDistance > 0) DRIVE_forward(PID, secondDistance, secondSpeed);
		else if(secondDistance < 0) DRIVE_backwards(PID, fabs(secondDistance), secondSpeed);
		if(coneIntakeRetract) GOLIATH_pickUp(false, 100);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
}

void FAST_deposit(ENUM_driveMode mode, float firstDistance, float firstSpeed, int firstTimeout, float secondDistance, float secondSpeed, int secondTimeout, bool driveRectify){

	resetValues();
	LOADED_mobileGoal(false, false, false, false);
	LOADED_arm(true);
	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	drive.motionProfile.distanceMultiplier[0] = 0.25;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = (int)(40);
	drive.motionProfile.offsets[1] = (int)(firstSpeed);
	drive.rectify = driveRectify;
	writeDebugStream("Before First loop\n\n");
	for(int C = 0; drive.PID.notDone && C<(int)(firstTimeout/4); C++){
		if(firstDistance >= 0) DRIVE_forward(MtnPrfl, (float)(firstDistance/4), firstSpeed);
		else if(firstDistance < 0) DRIVE_backwards(MtnPrfl, (fabs(firstDistance)/4), firstSpeed);
		motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = -127;
		ARM_move(PID, 1);
		MOBILEGOAL_retract(false);
		delay(META_loopsDelay);
		writeDebugStream("FirstLoop\n\n");
	}

	resetValues();
	drive.rectify = driveRectify;
	for(int C = 0; (drive.PID.notDone || mobileGoalIntake.PID.notDone) && C<firstTimeout; C++){
		if(firstDistance > 0) DRIVE_forward(PID, firstDistance, firstSpeed);
		else if(firstDistance < 0) DRIVE_backwards(PID, fabs(firstDistance), firstSpeed);
		MOBILEGOAL_retract(false);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
		writeDebugStream("SecondLoop\n\n");
	}

	resetValues();
	LOADED_mobileGoal(true, false, false, false);
	LOADED_arm(true);
	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	drive.motionProfile.distanceMultiplier[0] = 0.25;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = 40;
	drive.motionProfile.offsets[1] = (int)(secondSpeed);
	drive.rectify = driveRectify;
	for(int C = 0; drive.PID.notDone && C<(int)(secondTimeout/4); C++){
		if(secondDistance > 0) DRIVE_forward(MtnPrfl, secondDistance/5, secondSpeed);
		else if(secondDistance < 0) DRIVE_backwards(MtnPrfl, fabs(secondDistance)/5, secondSpeed);
		ARM_move(PID, 1);
		MOBILEGOAL_retract(false);
		delay(META_loopsDelay);
	}

	resetValues();
	LOADED_mobileGoal(true, true, false, false);
	drive.rectify = driveRectify;
	for(int C = 0; drive.PID.notDone && C<secondTimeout; C++){
		if(secondDistance > 0) DRIVE_forward(PID, secondDistance, secondSpeed);
		else if(secondDistance < 0) DRIVE_backwards(PID, fabs(secondDistance), secondSpeed);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();

}

#endif
