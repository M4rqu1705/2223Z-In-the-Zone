#ifndef functions.h
#define functions.h

#pragma systemfile

short lastDriveEncoderLValue, lastDriveEncoderRValue, lastArmPotLValue, lastArmPotRValue;
byte cycleTime, clawOutput;
static void initialize(){
	SensorValue[driveEncoderL] = 0;
	SensorValue[driveEncoderR] = 0;
	short lastDriveEncoderLValue = 0, lastDriveEncoderRValue = 0, lastArmPotLValue = 0, lastArmPotRValue = 0;
	byte cycleTime = 15;
}

static void printSpeed(){
	writeDebugStreamLine("move(%f, %f, %f, %f, %f);",
	(SensorValue[driveEncoderL]-lastDriveEncoderLValue)/cycleTime,
	(SensorValue[driveEncoderR]-lastDriveEncoderRValue)/cycleTime,
	(SensorValue[armPotL]-lastArmPotLValue)/cycleTime,
	(SensorValue[armPotR]-lastArmPotRValue)/cycleTime,
	clawOutput);
}

static void prepareForNextLoop(){
	lastDriveEncoderLValue = SensorValue[driveEncoderL];
	lastDriveEncoderRValue = SensorValue[driveEncoderR];
	lastArmPotLValue = SensorValue[armPotL];
	lastArmPotRValue = SensorValue[armPotR];
}

#endif
