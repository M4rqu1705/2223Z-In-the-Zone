#ifndef UTILS_H_
#define UTILS_H_

#pragma systemfile

byte MATH_clamp(float in){
	if(in>=127) return 127;
	else if(in<=-127) return -127;
	else return (byte)(in);

}

bool MATH_withinThreshold(float inNumber, float max, float min){
	return (inNumber < max && inNumber > min);
}

//Variable "initialization"
void slewRateInit(STRUCT_slewRate &out, byte _presentCurrent, int _presentCycle, byte _desiredCurrent, int _desiredCycle){
	out.presentCurrent = _presentCurrent;	out.presentCycle = _presentCycle;
	out.desiredCurrent = _desiredCurrent;	out.desiredCycle = _desiredCycle;
}

void pidInit(STRUCT_PID &out, float _kP, float _kI, float _kD, int _error, float _integral, float _derivative, int _lastError, int _output){
	out.KP = _kP;	out.KI = _kI;	out.KD = _kD;
	out.error = _error;	out.integral = _integral;	out.derivative = _derivative;
	out.lastError = _lastError;
	out.output = _output;
}

void slewRateControl(tMotor port, STRUCT_slewRate in){
	motor[port] = MATH_clamp(((in.desiredCurrent-in.presentCurrent)/(in.desiredCycle - in.presentCycle))*(in.presentCycle+1) + in.presentCurrent);
}

//Adjust PID constants based on load and desired action
void LOADED_mobileGoal(bool loaded, bool retract, bool usingGyro, bool speedPID){
	//If Mobile Goal is loaded
	if(loaded){
		//If Mobile Goal Intake will retract
		if(retract){

		}
		//If Mobile Goal Intake will extend
		else{

		}
		//If drive will rotate, so it will use gyro
		if(usingGyro){
			pidInit(drive.left.PID, PID_KPdriveGyro[1], PID_KIdriveGyro[1], PID_KDdriveGyro[1], 0, 0, 0, 0, 0);
			pidInit(drive.right.PID, PID_KPdriveGyro[1], PID_KIdriveGyro[1], PID_KDdriveGyro[1], 0, 0, 0, 0, 0);
		}
		//If drive will use speed PID to translate
		else if(speedPID){
			pidInit(drive.left.PID, PID_KPdrive[1][1], PID_KIdrive[1][1], PID_KPdrive[1][1], 0, 0, 0, 0, 0);
			pidInit(drive.right.PID, PID_KPdrive[1][1], PID_KIdrive[1][1], PID_KPdrive[1][1], 0, 0, 0, 0, 0);
		}
		//If drive will use position PID
		else{
			pidInit(drive.left.PID, PID_KPdrive[1][0], PID_KIdrive[1][0], PID_KPdrive[1][0], 0, 0, 0, 0, 0);
			pidInit(drive.right.PID, PID_KPdrive[1][0], PID_KIdrive[1][0], PID_KPdrive[1][0], 0, 0, 0, 0, 0);
		}
	}
	//If Mobile Goal is not loaded
	else{
		if(retract){

		}
		//If Mobile Goal Intake will extend
		else{

		}
		//If drive will rotate, so it will use gyro
		if(usingGyro){
			pidInit(drive.left.PID, PID_KPdriveGyro[0], PID_KIdriveGyro[0], PID_KDdriveGyro[0], 0, 0, 0, 0, 0);
			pidInit(drive.right.PID, PID_KPdriveGyro[0], PID_KIdriveGyro[0], PID_KDdriveGyro[0], 0, 0, 0, 0, 0);
		}
		//If drive will use speed PID to translate
		else if(speedPID){
			pidInit(drive.left.PID, PID_KPdrive[0][1], PID_KIdrive[0][1], PID_KPdrive[0][1], 0, 0, 0, 0, 0);
			pidInit(drive.right.PID, PID_KPdrive[0][1], PID_KIdrive[0][1], PID_KPdrive[0][1], 0, 0, 0, 0, 0);
		}
		//If drive will use position PID
		else{
			pidInit(drive.left.PID, PID_KPdrive[0][0], PID_KIdrive[0][0], PID_KPdrive[0][0], 0, 0, 0, 0, 0);
			pidInit(drive.right.PID, PID_KPdrive[0][0], PID_KIdrive[0][0], PID_KPdrive[0][0], 0, 0, 0, 0, 0);
		}
	}
}

#endif
