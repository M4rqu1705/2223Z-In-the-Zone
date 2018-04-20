#ifndef UTILS_H_
#define UTILS_H_

#pragma systemfile

byte clamp(float in){
	if(in>=127) return 127;
	else if(in<=-127) return -127;
	else return (byte)(in);
}

float map(float inNumber, float inMax, float inMin, float outMax, float outMin){
	return (float)((inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
	//Retrieved from https://www.arduino.cc/reference/en/language/functions/math/map
}

bool isWithinThreshold(float inNumber, float max, float min){
	return (inNumber < max && inNumber > min);
}

float calculateSpeed(float &_previousValue, float _currentValue, int _pulsesPerRevolution){
	float currentSpeed = ((_currentValue-_previousValue)/(META_loopsDelay*0.001)/_pulsesPerRevolution)*60;
	_previousValue = _currentValue;
	return currentSpeed;
}

//Conversions
int convertInchesToPulses(float inches){
	return round(inches*(360/(META_driveWheelDiameter*PI)));
}

int convertDegreesToPulses(float targetDegrees, float turnRadius){
	return convertInchesToPulses((targetDegrees*PI*turnRadius)/180);
	//return (angle*PI*r)/180
}

float convertSwingTurnInside(float turnRadius, float speed){
	return (-(META_driveWidth/turnRadius) * speed + speed);
}

int convertDegreesToTicks(float targetDegrees){
	return (int)round(targetDegrees*10);
}

//Slew rate control
void slewRateControl(tMotor port, STRUCT_slewRate in){
	float slope = ((in.desiredCurrent-in.presentCurrent)/(in.desiredCycle - in.presentCycle));
	if(slope > in.maxSlope) slope = in.maxSlope;
	else if (slope < -in.maxSlope) slope = -in.maxSlope;

	motor[port] = clamp(round(slope *(in.presentCycle) + in.presentCurrent));
}

int updateEncoder(STRUCT_SENSOR_encoder &out){
	if(out.inverted) out.currentPosition = -SensorValue[out.port];
	else out.currentPosition = SensorValue[out.port];
	out.RPM = calculateSpeed((float)(out.previousPosition), (float)(out.currentPosition), out.pulsesPerRevolution);

	return out.currentPosition;
}

//Variable "initialization"
//SlewRate control variable "initialization"
void slewRateInit(STRUCT_slewRate &out, byte _presentCurrent, byte _desiredCurrent, int _presentCycle, int _desiredCycle, float _maxSlope){
	out.presentCurrent = _presentCurrent;
	out.desiredCurrent = _desiredCurrent;
	out.presentCycle = _presentCycle;
	out.desiredCycle = _desiredCycle;
	out.maxSlope = _maxSlope;
}

//PID variable initialization
void pidInit(STRUCT_PID &out, float _kP, float _kI, float _kD, int _error, float _integral, float _derivative, int _lastError, byte _output, byte _correctionThreshold){
	out.KP = _kP;	out.KI = _kI;	out.KD = _kD;
	out.error = _error;	out.integral = _integral;	out.derivative = _derivative;
	out.lastError = _lastError;
	out.output = _output;
	out.correctionThreshold = _correctionThreshold;
}

//Calculate PID output based on constants, setpoint and sensor values
byte calculatePID(STRUCT_PID &values,  int setpoint,  int processVariable){

	//Calculate Error
	values.error = (int)(setpoint - processVariable);

	//Calculate integral
	values.integral += values.error*(META_loopsDelay*0.001);
	//Reset Integral to 0 if is within the accepted 'done' range
	if(isWithinThreshold(values.error, values.correctionThreshold, -values.correctionThreshold)) values.integral = 0;

	//Calculate Derivative
	values.derivative = (values.error - values.lastError)/(META_loopsDelay * 0.001);

	//Calculate output
	values.output = clamp((values.KP * values.error) +	(values.KI * values.integral) +	(values.KD * values.derivative));

	//Assign error to last error
	values.lastError = values.error;

	//Just in case Debug is needed
	//writeDebugStreamLine("values = %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", values.KP, values.KI, values.KD, values.integralMax, values.error, values.lastError, values.integral, values.cyclesCounter, values.timeoutCounter, values.output); datalogAddValue(0, processVariable);

	return (values.output);
}

//Adjust PID constants based on load and desired action
void mobileGoalIsLoaded(bool loaded, bool retract, bool usingGyro, bool speedPID){
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
			pidInit(drive.left.PID, PID_KPdriveGyro[1], PID_KIdriveGyro[1], PID_KDdriveGyro[1], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
			pidInit(drive.right.PID, PID_KPdriveGyro[1], PID_KIdriveGyro[1], PID_KDdriveGyro[1], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
		}
		//If drive will use speed PID to translate
		else if(speedPID){
			pidInit(drive.left.PID, PID_KPdrive[1][1], PID_KIdrive[1][1], PID_KPdrive[1][1], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
			pidInit(drive.right.PID, PID_KPdrive[1][1], PID_KIdrive[1][1], PID_KPdrive[1][1], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
		}
		//If drive will use position PID
		else{
			pidInit(drive.left.PID, PID_KPdrive[1][0], PID_KIdrive[1][0], PID_KPdrive[1][0], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
			pidInit(drive.right.PID, PID_KPdrive[1][0], PID_KIdrive[1][0], PID_KPdrive[1][0], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
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
			pidInit(drive.left.PID, PID_KPdriveGyro[0], PID_KIdriveGyro[0], PID_KDdriveGyro[0], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
			pidInit(drive.right.PID, PID_KPdriveGyro[0], PID_KIdriveGyro[0], PID_KDdriveGyro[0], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
		}
		//If drive will use speed PID to translate
		else if(speedPID){
			pidInit(drive.left.PID, PID_KPdrive[0][1], PID_KIdrive[0][1], PID_KPdrive[0][1], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
			pidInit(drive.right.PID, PID_KPdrive[0][1], PID_KIdrive[0][1], PID_KPdrive[0][1], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
		}
		//If drive will use position PID
		else{
			pidInit(drive.left.PID, PID_KPdrive[0][0], PID_KIdrive[0][0], PID_KPdrive[0][0], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
			pidInit(drive.right.PID, PID_KPdrive[0][0], PID_KIdrive[0][0], PID_KPdrive[0][0], 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
		}
	}
}

#endif
