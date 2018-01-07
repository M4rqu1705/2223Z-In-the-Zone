#ifndef fullRobotTestFunctions.h
#define fullRobotTestFunctions.h

#pragma systemfile

//**======================================================**Previously on .h files**======================================================**//

typedef struct{
	bool buttonPressed[2];     //Boolean to store if the joystick drive invert button was pressed in the previous cycle. Used in driver control
	signed byte state;             //Variable to store the current state of the button meaning either direction, retract or others

}joystickToggleButtonStruct;

enum direction {Forward = 0, Backward, TurnLeft, TurnRight };
typedef struct{
	joystickToggleButtonStruct directionNormal;   //Struct to toggle change direction button
	bool notDone;                     //Variable to indicate if the drive is done or not. Used in autonomous
	unsigned byte counter;            //Variable to count asynchronously to check if drive is done or not. Used in autonomous
	float PID[10];                    //Array to store values to pass to the PID function. Used in autonomous
	signed byte joystickInputs[2];    //Array to store the joystick analgo inputs. 0 is forward input, 1 is side input. Used to not call for the current joystick values various times
	double slewRateOutputs[2];        //Array to store the slew increase or decrease. Used in operator control
	signed byte outputs[2];           //Array to store the future outputs of the motors. 0 is left, 1 is right. Used in both autonomous and user control
}robotDriveStruct;

typedef struct{
	joystickToggleButtonStruct retract;   //Struct to toggle retract buttons
	bool notDone;             //Boolean to indicate if the mobile goal intake is done moving or not. Used in operator control
	unsigned byte counter;    //Variable to count asynchronously to check if the intake is done or not. Used in autonomous
	double PID[10];           //Array to store the values to pass to the PID function. Used in autonomous and operator control
	signed byte output;       //Variable to store the future output of the intake of the Mobile Goal. Used in both autonomous and operator control
}robotMobileGoalIntakeStruct;

typedef struct{
	joystickToggleButtonStruct armUp;
	bool notDone;
	unsigned byte counter;
	double PID[10];
	signed byte joystickInput;
	signed byte output;
}robotArmStruct;

robotDriveStruct drive;
robotMobileGoalIntakeStruct mobileGoalIntake;
robotArmStruct arm;

//Function prototypes
void initialize();
void resetValues();

void driveOperatorControl(bool test = false);
void move(direction orientation, float pulses, signed byte speed);

void moveArm(unsigned byte position);
void armOperatorControl(bool test=false, bool simple, bool analog = false);

void moveMobileGoal(bool retract);
void mobileGoalOperatorControl(bool test=false, bool simple);

//LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD//
//#include "fullRobotLCD.h"
#include "fullRobotLCDCalibrate.h"

//**======================================================**Previously on .c files**======================================================**//

#include "fullRobotTestUtils.h"

//Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset//

//Initialize--Initialize--Initialize--Initialize--Initialize--Initialize--Initialize--Initialize--Initialize--Initialize--Initialize//
void initialize(){
#ifdef META_usingLCD
	lcdInit();
#endif

	//Initialize all motors and sensors using constants from constants.h
	motorType[MOTOR_driveLB] = MOTORTYPE_driveLB;
	motorType[MOTOR_claw] = MOTORTYPE_claw;
	motorType[MOTOR_mogoIntake] = MOTORTYPE_mogoIntake;
	motorType[MOTOR_driveLF] = MOTORTYPE_driveLF;
	motorType[MOTOR_driveLM] = MOTORTYPE_driveLM;
	motorType[MOTOR_driveRM] = MOTORTYPE_driveRM;
	motorType[MOTOR_driveRF] = MOTORTYPE_driveRF;
	motorType[MOTOR_arm] = MOTORTYPE_arm;
	motorType[MOTOR_driveRB] = MOTORTYPE_driveRB;

	bMotorReflected[MOTOR_driveLB] = MOTORINVERT_driveLB;
	bMotorReflected[MOTOR_claw] = MOTORINVERT_claw;
	bMotorReflected[MOTOR_mogoIntake] = MOTORINVERT_mogoIntake;
	bMotorReflected[MOTOR_driveLF] = MOTORINVERT_driveLF;
	bMotorReflected[MOTOR_driveLM] = MOTORINVERT_driveLF;
	bMotorReflected[MOTOR_driveRM] = MOTORINVERT_driveRM;
	bMotorReflected[MOTOR_driveRF] = MOTORINVERT_driveRF;
	bMotorReflected[MOTOR_arm] = MOTORINVERT_arm;
	bMotorReflected[MOTOR_driveRB] = MOTORINVERT_driveRB;

	SensorType[SENSOR_powerExpanderStatus] = sensorAnalog;
	SensorType[SENSOR_potMogo] = sensorPotentiometer;
	SensorType[SENSOR_potArm] = sensorPotentiometer;
	SensorType[SENSOR_encoderL] = sensorQuadEncoder;
	SensorType[SENSOR_encoderR] = sensorQuadEncoder;

	resetValues();
	clearDebugStream();
	datalogClear();
}

void resetValues() {
	//Stop all motors
	motor[1] = motor[2] = motor[3] = motor[4] = motor[5] = motor[6] = motor[7] = motor[8] = motor[9] = motor[10] = 0;

	drive.directionNormal.buttonPressed[0] = false;
	drive.directionNormal.buttonPressed[1] = false;
	drive.directionNormal.state = 1;
	drive.notDone = true;
	drive.counter = 0;

	drive.PID[0] = PID_KPdrive;
	drive.PID[1] = PID_KIdrive;
	drive.PID[2] = PID_KDdrive;
	drive.PID[3] = PID_integralMaxDrive;
	drive.PID[4] = 0;
	drive.PID[5] = 0;
	drive.PID[6] = 0;
	drive.PID[7] = PID_correctionCyclesDrive;
	drive.PID[8] = PID_doneThresholdDrive;
	drive.PID[9] = 0;	//PID output
	drive.joystickInputs[0] = 0;
	drive.joystickInputs[1] = 0;
	drive.slewRateOutputs[0] = 0;
	drive.slewRateOutputs[1] = 0;
	drive.outputs[0] = 0;
	drive.outputs[0] = 0;

	arm.armUp.buttonPressed[0] = false;
	arm.armUp.buttonPressed[1] = false;
	arm.armUp.state = 0;
	arm.notDone = true;
	arm.counter = 0;

	arm.PID[0] = PID_KParm;
	arm.PID[1] = PID_KIarm;
	arm.PID[2] = PID_KDarm;
	arm.PID[3] = PID_integralMaxArm;
	arm.PID[4] = 0;
	arm.PID[5] = 0;
	arm.PID[6] = 0;
	arm.PID[7] = PID_correctionCyclesArm;
	arm.PID[8] = PID_doneThresholdArm;
	arm.PID[9] = 0;	//PID output

	arm.output = 0;

	mobileGoalIntake.retract.buttonPressed[0] = false;
	mobileGoalIntake.retract.buttonPressed[1] = false;
	mobileGoalIntake.retract.state = 0;
	mobileGoalIntake.notDone = true;
	mobileGoalIntake.counter = 0;

	mobileGoalIntake.PID[0] = PID_KPmogoExtend;
	mobileGoalIntake.PID[1] = PID_KImogoExtend;
	mobileGoalIntake.PID[2] = PID_KDmogoExtend;
	mobileGoalIntake.PID[3] = PID_integralMaxMogoExtend;
	mobileGoalIntake.PID[4] = 0;
	mobileGoalIntake.PID[5] = 0;
	mobileGoalIntake.PID[6] = 0;
	mobileGoalIntake.PID[7] = PID_correctionCyclesMogoExtend;
	mobileGoalIntake.PID[8] = PID_doneThresholdMogoExtend;
	mobileGoalIntake.PID[9] = 0;	//PID output

	mobileGoalIntake.output = 0;

}

//Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive//
void driveOperatorControl(bool test){
	if(test){
		drive.joystickInputs[0] = vexRT[JOYSTICK_driveF];
		drive.joystickInputs[1] = vexRT[JOYSTICK_driveS];

		//Calculate "arcade drive" values for left and right side
		drive.outputs[0] = CLAMP(ROUND(-drive.joystickInputs[0] + drive.joystickInputs[1]));
		drive.outputs[1] = CLAMP(ROUND(-drive.joystickInputs[0] - drive.joystickInputs[1]));

		//Move motors using calculated values for left and right side
		motor[MOTOR_driveLF] = motor[MOTOR_driveLB] = drive.outputs[0];
		motor[MOTOR_driveRF] = motor[MOTOR_driveRB] = drive.outputs[1];
		motor[MOTOR_driveLM] = (drive.outputs[0]*0.75);
		motor[MOTOR_driveRM] = (drive.outputs[1]*0.75);
	}
	else{
		//Button toggle
		if ((bool)(vexRT[JOYSTICK_driveInvert])){
			if (!drive.directionNormal.buttonPressed[0]) {
				drive.directionNormal.buttonPressed[0] = true;
				if(drive.directionNormal.state == 0) drive.directionNormal.state = 1;
				else drive.directionNormal.state = 0;
			}
		}
		else drive.directionNormal.buttonPressed[0] = false;

		//Assign joystick values to variables
		drive.joystickInputs[0] = vexRT[JOYSTICK_driveF];
		drive.joystickInputs[1] = vexRT[JOYSTICK_driveS];

		//Only use values if withing threshold. If not withing threshold, assign 0
		if (drive.joystickInputs[0] < META_driveOpControlThreshold && drive.joystickInputs[0] > -META_driveOpControlThreshold) drive.joystickInputs[0] = 0;
		if (drive.joystickInputs[1] < META_driveOpControlThreshold && drive.joystickInputs[1] > -META_driveOpControlThreshold) drive.joystickInputs[1] = 0;

		//Slewrate control. Assign value to output incrementing or decreasing values using SLEW_GAIN
		if (drive.slewRateOutputs[0] + META_slewGain < drive.joystickInputs[0]+META_slewGainThreshold) drive.slewRateOutputs[0] += META_slewGain;
		else if (drive.slewRateOutputs[0] - META_slewGain > drive.joystickInputs[0]-META_slewGainThreshold)	drive.slewRateOutputs[0] -= META_slewGain;
		else if (drive.joystickInputs[0] == 0) drive.slewRateOutputs[0] = 0;

		if (drive.slewRateOutputs[1] + META_slewGain < drive.joystickInputs[1]+2) drive.slewRateOutputs[1] += META_slewGain;
		else if (drive.slewRateOutputs[1] - META_slewGain > drive.joystickInputs[1]-META_slewGainThreshold) drive.slewRateOutputs[1] -= META_slewGain;
		else if (drive.joystickInputs[1] == 0) drive.slewRateOutputs[1] = 0;

		drive.slewRateOutputs[0] = CLAMP(drive.slewRateOutputs[0]);
		drive.slewRateOutputs[1] = CLAMP(drive.slewRateOutputs[1]);

		if(drive.directionNormal.state == 1){
			//Calculate "arcade drive" values for left and right side
			drive.outputs[0] = CLAMP(ROUND(-drive.slewRateOutputs[0] + drive.slewRateOutputs[1]));
			drive.outputs[1] = CLAMP(ROUND(-drive.slewRateOutputs[0] - drive.slewRateOutputs[1]));
		}
		else{
			drive.outputs[0] = CLAMP(ROUND(drive.slewRateOutputs[0] + drive.slewRateOutputs[1]));
			drive.outputs[1] = CLAMP(ROUND(drive.slewRateOutputs[0] - drive.slewRateOutputs[1]));
		}

		//Move motors using calculated values for left and right side
		motor[MOTOR_driveLF] = motor[MOTOR_driveLB] = drive.outputs[0];
		motor[MOTOR_driveRF] = motor[MOTOR_driveRB] = drive.outputs[1];
		motor[MOTOR_driveLM] = (drive.outputs[0]*0.75);
		motor[MOTOR_driveRM] = (drive.outputs[1]*0.75);
	}
}

void move(direction orientation, float pulses, signed byte speed) {
	//Recalculate pulses to convert them to degrees of rotation
	if (orientation == TurnLeft || orientation == TurnRight) pulses = DEGREEStoPULSES(pulses);
	//Recaluclate pulses to convert them to inches of movement
	else if(orientation == Forward || orientation == Backward) pulses = INCHEStoPULSES(pulses);

	//Calculate PID and rectify robot if necessary
	calculatePID(drive.PID, pulses, ((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR])) / 2));
	//Rectify with encoders only if moving forward or backwards
	if(orientation == Forward || orientation == Backward) rectifyOutputsEncoder(drive.outputs, drive.PID[9], abs(SensorValue[SENSOR_encoderL]), abs(SensorValue[SENSOR_encoderR]));
	else {
		drive.outputs[0] = drive.PID[9];
		drive.outputs[1] = drive.PID[9];
	}

	//Make sure left and right orientation values are within a range of values between -speed to speed
	drive.outputs[0] = MAP(drive.outputs[0], 127, -127, speed, -speed);
	drive.outputs[1] = MAP(drive.outputs[1], 127, -127, speed, -speed);

	//Move motors based on PID values, direction in which to move
	switch (orientation) {
	case Forward:
		motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.outputs[0];
		motor[MOTOR_driveRF] = motor[MOTOR_driveLM] = motor[MOTOR_driveRB] = drive.outputs[1];
		break;

	case Backward:
		motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = -drive.outputs[0];
		motor[MOTOR_driveRF] = motor[MOTOR_driveLM] = motor[MOTOR_driveRB] = -drive.outputs[1];
		break;

	case TurnLeft:
		motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = -drive.outputs[0];
		motor[MOTOR_driveRF] = motor[MOTOR_driveLM] = motor[MOTOR_driveRB] = drive.outputs[1];
		break;

	case TurnRight:
		motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.outputs[0];
		motor[MOTOR_driveRF] = motor[MOTOR_driveLM] = motor[MOTOR_driveRB] = -drive.outputs[1];
		break;

	}

	//Check if drive is done
	checkIfDriveDone(drive);
}
//Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm//
void armOperatorControl(bool test, bool simple, bool analog){
	if(test){
		if(analog){
			motor[MOTOR_arm] = vexRT[JOYSTICK_armAnalog];

			if(vexRT[JOYSTICK_clawOpen] == 1){
				motor[MOTOR_claw] = META_clawSpeed;
			}
			else if(vexRT[JOYSTICK_clawClose] == 1){
				motor[MOTOR_claw] = -META_clawSpeed;
			}
			else motor[MOTOR_claw] = 0;
		}
		else{
			if(vexRT[JOYSTICK_armUp] == 1) motor[MOTOR_arm] = 127;
			else if(vexRT[JOYSTICK_armDown] == 1) motor[MOTOR_arm] = -127;
			else motor[MOTOR_arm] = 0;
		}
	}
	else if(simple){
		if(analog){
			arm.joystickInput = vexRT[JOYSTICK_armAnalog];
			if(arm.joystickInput <= META_armOpControlThreshold && arm.joystickInput >= META_armOpControlThreshold) arm.joystickInput = 0;

			if(SensorValue[SENSOR_potArm] > META_armDown && SensorValue[SENSOR_potArm] <= META_armUp) motor[MOTOR_arm] = arm.joystickInput;
			else if(SensorValue[SENSOR_potArm] <= META_armDown && arm.joystickInput>0) motor[MOTOR_arm] = arm.joystickInput;
			else if(SensorValue[SENSOR_potArm] > META_armUp && arm.joystickInput < 0) motor[MOTOR_arm] = arm.joystickInput;
			else motor[MOTOR_arm] = 0;

			if(vexRT[JOYSTICK_clawOpen] == 1){
				motor[MOTOR_claw] = META_clawSpeed;
			}
			else if(vexRT[JOYSTICK_clawClose] == 1){
				motor[MOTOR_claw] = -META_clawSpeed;
			}
			else motor[MOTOR_claw] = 0;
		}
		else{
			if(vexRT[JOYSTICK_armUp] == 1){
				if(SensorValue[SENSOR_potArm] < META_armUp) motor[MOTOR_arm] = 127;
				else motor[MOTOR_arm] = 0;
			}
			else if(vexRT[JOYSTICK_armDown] == 1){
				if(SensorValue[SENSOR_potArm] > META_armDown) motor[MOTOR_arm] = -127;
				else motor[MOTOR_arm] = 0;
			}
			else motor[MOTOR_arm] = 0;
		}
	}
	else{
		if(analog){
			arm.joystickInput = vexRT[JOYSTICK_armAnalog];
			if(arm.joystickInput <= META_armOpControlThreshold && arm.joystickInput >= META_armOpControlThreshold) arm.joystickInput = 0;

			if(arm.joystickInput == 0){
				arm.PID[0] = PID_KParm;
				arm.PID[1] = PID_KIarm;
				arm.PID[2] = PID_KDarm;
				arm.PID[3] = PID_integralMaxArm;
				arm.PID[4] = 0;
				arm.PID[5] = 0;
				arm.PID[6] = 0;
				arm.PID[7] = PID_correctionCyclesArm;
				arm.PID[8] = PID_doneThresholdArm;
				arm.PID[9] = 0;	//PID output

				motor[MOTOR_arm] = 0;
			}
			else if(arm.joystickInput > 0){
				calculatePID(arm.PID, META_armUp, SensorValue[SENSOR_potArm]);
				arm.PID[9] = MAP(arm.PID[9], 127, -127, abs(arm.joystickInput), -abs(arm.joystickInput));
				motor[MOTOR_arm] = arm.PID[9];
			}
			else if(arm.joystickInput < 0){
				calculatePID(arm.PID, META_armDown, SensorValue[SENSOR_potArm]);
				arm.PID[9] = MAP(arm.PID[9], 127, -127, abs(arm.joystickInput), -abs(arm.joystickInput));
				motor[MOTOR_arm] = arm.PID[9];
			}
		}
		else{

			if(vexRT[JOYSTICK_armUp] == 1) {
				if (!arm.armUp.buttonPressed[1]) {
					arm.armUp.buttonPressed[1] = true;
					arm.armUp.state = 1;

					arm.PID[0] = PID_KParm;
					arm.PID[1] = PID_KIarm;
					arm.PID[2] = PID_KDarm;
					arm.PID[3] = PID_integralMaxArm;
					arm.PID[4] = 0;
					arm.PID[5] = 0;
					arm.PID[6] = 0;
					arm.PID[7] = PID_correctionCyclesArm;
					arm.PID[8] = PID_doneThresholdArm;
					arm.PID[9] = 0;	//PID output
				}
				calculatePID(arm.PID, META_armUp, SensorValue[SENSOR_potArm]);
				motor[MOTOR_arm] = arm.PID[9];
			}
			else if(vexRT[JOYSTICK_armDown] == 1) {
				if (!arm.armUp.buttonPressed[0]) {
					arm.armUp.buttonPressed[0] = true;
					arm.armUp.state = 0;

					arm.PID[0] = PID_KParm;
					arm.PID[1] = PID_KIarm;
					arm.PID[2] = PID_KDarm;
					arm.PID[3] = PID_integralMaxArm;
					arm.PID[4] = 0;
					arm.PID[5] = 0;
					arm.PID[6] = 0;
					arm.PID[7] = PID_correctionCyclesArm;
					arm.PID[8] = PID_doneThresholdArm;
					arm.PID[9] = 0;	//PID output
				}
				calculatePID(arm.PID, META_armDown, SensorValue[SENSOR_potArm]);
				motor[MOTOR_arm] = arm.PID[9];
			}
			else {
				arm.armUp.buttonPressed[0] = false;
				arm.armUp.buttonPressed[1] = false;
				motor[MOTOR_arm] = 0;
			}
		}
	}
}

void moveArm(unsigned byte position){

}

//Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake//

void mobileGoalOperatorControl(bool test, bool simple){
	if(test){
		if(vexRT[JOYSTICK_mogoExtend] == 1) motor[MOTOR_mogoIntake] = META_mogoMaxOutput;
		else if(vexRT[JOYSTICK_mogoRetract] == 1) motor[MOTOR_mogoIntake] = -META_mogoMaxOutput;
		else motor[MOTOR_mogoIntake] = 0;
	}

	else if(simple){
		if(vexRT[JOYSTICK_mogoExtend] == 1){
			if(SensorValue[SENSOR_potMogo] < META_mogoExtended) motor[MOTOR_mogoIntake] = ROUND(META_mogoMaxOutput*0.9);
			else motor[MOTOR_mogoIntake] = 0;
		}
		else if(vexRT[JOYSTICK_mogoRetract] == 1){
			if(SensorValue[SENSOR_potMogo] > META_mogoRetracted) motor[MOTOR_mogoIntake] = -ROUND(META_mogoMaxOutput*0.9);
			else motor[MOTOR_mogoIntake] = 0;
		}
		else motor[MOTOR_mogoIntake] = 0;
	}
	else{
		if(vexRT[JOYSTICK_mogoExtend] == 1) {
			if (!mobileGoalIntake.retract.buttonPressed[1]){
				mobileGoalIntake.retract.buttonPressed[1] = true;
				//mobileGoalIntake.retract.state = 1;

				mobileGoalIntake.PID[0] = PID_KPmogoExtend;
				mobileGoalIntake.PID[1] = PID_KImogoExtend;
				mobileGoalIntake.PID[2] = PID_KDmogoExtend;
				mobileGoalIntake.PID[3] = PID_integralMaxMogoExtend;
				mobileGoalIntake.PID[4] = 0;
				mobileGoalIntake.PID[5] = 0;
				mobileGoalIntake.PID[6] = 0;
				mobileGoalIntake.PID[7] = PID_correctionCyclesMogoExtend;
				mobileGoalIntake.PID[8] = PID_doneThresholdMogoExtend;
				mobileGoalIntake.PID[9] = 0;	//PID output
			}
			calculatePID(mobileGoalIntake.PID, META_mogoExtended, SensorValue[SENSOR_potMogo]);
			motor[MOTOR_mogoIntake] = mobileGoalIntake.PID[9];
		}
		else if(vexRT[JOYSTICK_mogoRetract] == 1) {
			if (!mobileGoalIntake.retract.buttonPressed[0]) {
				mobileGoalIntake.retract.buttonPressed[0] = true;
				mobileGoalIntake.retract.state = 0;

				mobileGoalIntake.PID[0] = PID_KPmogoRetract;
				mobileGoalIntake.PID[1] = PID_KImogoRetract;
				mobileGoalIntake.PID[2] = PID_KDmogoRetract;
				mobileGoalIntake.PID[3] = PID_integralMaxMogoRetract;
				mobileGoalIntake.PID[4] = 0;
				mobileGoalIntake.PID[5] = 0;
				mobileGoalIntake.PID[6] = 0;
				mobileGoalIntake.PID[7] = PID_correctionCyclesMogoRetract;
				mobileGoalIntake.PID[8] = PID_correctionCyclesMogoRetract;
				mobileGoalIntake.PID[9] = 0; //PID output
			}
			calculatePID(mobileGoalIntake.PID, META_mogoRetracted, SensorValue[SENSOR_potMogo]);
			motor[MOTOR_mogoIntake] = mobileGoalIntake.PID[9];
		}
		else {
			mobileGoalIntake.retract.buttonPressed[0] = false;
			mobileGoalIntake.retract.buttonPressed[1] = false;
			motor[MOTOR_mogoIntake] = 0;
		}
	}
}

void moveMobileGoal(bool retract){
	if(retract){
		calculatePID(mobileGoalIntake.PID, META_mogoRetracted, SensorValue[SENSOR_potMogo]);
	}
	else{
		calculatePID(mobileGoalIntake.PID, META_mogoExtended, SensorValue[SENSOR_potMogo]);
	}

	motor[MOTOR_mogoIntake] = mobileGoalIntake.PID[9];

	//Check if intake is done
	checkIfMogoDone(mobileGoalIntake);
}

#endif
