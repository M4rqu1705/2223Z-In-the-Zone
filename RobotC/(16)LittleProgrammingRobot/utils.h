#ifndef UTILS_H_
#define UTILS_H_

#pragma systemfile

//Scale values in different ranges
int MATH_map(int inNumber, int inMax, int inMin, int outMax, int outMin){
	return  (inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
	//Retrieved from https://www.arduino.cc/reference/en/language/functions/math/map/
}

//Clamp values to convert them into bytes, suitable for motor control
byte clamp(float in){
	if(in>=127) return 127;
	else if(in<=-127) return -127;
	else return (byte)(in);
}

//Check if values are within a limited threshold
bool checkWithinThreshold(float inNumber, float max, float min){
	return (inNumber < max && inNumber > min);
}

//Swap the two variable's values
void swap(float &firstElement, float &secondElement){
	float temp = firstElement;	firstElement = secondElement;	secondElement = temp;
}

//Conversions
short convertInchesToPulses(float inches){
	return round(inches*(360/(META_driveWheelDiameter*PI)));
}

short convertDegreesToPulses(float targetDegrees, float turnRadius){
	return convertInchesToPulses((targetDegrees*PI*turnRadius)/180);
	//return (angle*PI*r)/180
}

float calculateSwingTurnInsideSpeed(float turnRadius, float speed){
	return (-(META_driveWidth/turnRadius) * speed + speed);
}

float convertDegreesToTicks(float targetDegrees){
	return (float)(targetDegrees*10);
}

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

//Slew rate controller for motors
void slewRateControl(tMotor port, STRUCT_slewRate in){
	float slope = ((in.desiredCurrent-in.presentCurrent)/(in.desiredCycle - in.presentCycle));
	if(slope > in.maxSlope) slope = in.maxSlope;
	else if (slope < -in.maxSlope) slope = -in.maxSlope;

	motor[port] = clamp(ceil(slope *(in.presentCycle) + in.presentCurrent));
}

float calculateSpeed(float &_previousValue, float _currentValue, int _pulsesPerRevolution){
	float currentSpeed = ((_currentValue-_previousValue)/(META_loopsDelay*0.001)/_pulsesPerRevolution)*60;
	_previousValue = _currentValue;
	return currentSpeed;
}

int updateEncoder(STRUCT_SENSOR_encoder &out){
	if(out.inverted) out.currentPosition = -SensorValue[out.port];
	else out.currentPosition = SensorValue[out.port];
	out.RPM = calculateSpeed((float)(out.previousPosition), (float)(out.currentPosition), out.pulsesPerRevolution);

	return out.currentPosition;
}

float medianFilter(STRUCT_medianFilter &out, float toInsert){
	//Shift values to the right in order to make space for the value to be inserted and discard the last value
	for(int C = 4; C>0; C--)	swap(out.valuesHistory[C], out.valuesHistory[C-1]);
	//Insert new value into the first index, place where the oldest value is currently positioned
	out.valuesHistory[0] = toInsert;
	//Copy 'draft' to 'sorted collection'
	for(int C = 0; C<5; C++)	out.sorted[C] = out.valuesHistory[C];
	//Sort
	for(int C = 1; C<5; C++)
		for(int D = C; D>0 && out.sorted[D-1] > out.sorted[D]; D--)
		swap(out.sorted[D], out.sorted[D-1]);

	return out.sorted[2];
}

//PID calculation
byte calculatePID(STRUCT_PID &values,  int setPoint,  int processVariable){

	//Calculate Error
	values.error = (int)(setPoint - processVariable);

	//Calculate integral
	values.integral += values.error*(META_loopsDelay*0.001);
	//Reset Integral to 0 if is within the accepted 'done' range
	if(checkWithinThreshold(values.error, values.correctionThreshold, -values.correctionThreshold)) values.integral = 0;

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
void mobileGoalLoaded(bool loaded, bool retract, bool usingGyro, bool speedPID){
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
			pidInit(drive.left.PID, PID_KPdriveGyro[1], PID_KIdriveGyro[1], PID_KDdriveGyro[1], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
			pidInit(drive.right.PID, PID_KPdriveGyro[1], PID_KIdriveGyro[1], PID_KDdriveGyro[1], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
		}
		//If drive will use speed PID to translate
		else if(speedPID){
			pidInit(drive.left.PID, PID_KPdrive[1][1], PID_KIdrive[1][1], PID_KPdrive[1][1], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
			pidInit(drive.right.PID, PID_KPdrive[1][1], PID_KIdrive[1][1], PID_KPdrive[1][1], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
		}
		//If drive will use position PID
		else{
			pidInit(drive.left.PID, PID_KPdrive[1][0], PID_KIdrive[1][0], PID_KPdrive[1][0], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
			pidInit(drive.right.PID, PID_KPdrive[1][0], PID_KIdrive[1][0], PID_KPdrive[1][0], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
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
			pidInit(drive.left.PID, PID_KPdriveGyro[0], PID_KIdriveGyro[0], PID_KDdriveGyro[0], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
			pidInit(drive.right.PID, PID_KPdriveGyro[0], PID_KIdriveGyro[0], PID_KDdriveGyro[0], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
		}
		//If drive will use speed PID to translate
		else if(speedPID){
			pidInit(drive.left.PID, PID_KPdrive[0][1], PID_KIdrive[0][1], PID_KPdrive[0][1], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
			pidInit(drive.right.PID, PID_KPdrive[0][1], PID_KIdrive[0][1], PID_KPdrive[0][1], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
		}
		//If drive will use position PID
		else{
			pidInit(drive.left.PID, PID_KPdrive[0][0], PID_KIdrive[0][0], PID_KPdrive[0][0], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
			pidInit(drive.right.PID, PID_KPdrive[0][0], PID_KIdrive[0][0], PID_KPdrive[0][0], 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
		}
	}
}

#endif
