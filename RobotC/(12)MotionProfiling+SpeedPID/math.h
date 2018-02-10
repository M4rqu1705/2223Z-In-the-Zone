#ifndef MATH_H_
#define MATH_H_


#pragma systemfile

int MATH_map(int inNumber, int inMax, int inMin, int outMax, int outMin){
	return  (inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
	//Retrieved from https://www.arduino.cc/reference/en/language/functions/math/map/
}

signed byte MATH_clamp(signed int inNumber){
	if(inNumber>127) return 127;
	else if (inNumber<-127) return -127;
	else return (signed byte)(inNumber);
}

signed int MATH_round(float inNumber){
	if(inNumber>=0) return ((signed int)(ceil(inNumber-0.49)));
	else return ((signed int)(floor(inNumber+0.49)));
}

unsigned short MATH_inchesToPulses(float inches){
	return MATH_round(inches*(360/(META_driveWheelDiameter*PI)));
}

unsigned short MATH_degreesToPulses(float targetDegrees){
	return MATH_round(targetDegrees*META_driveWidth/META_driveWheelDiameter);
}

signed short MATH_degreesToTicks(float targetDegrees){
	return (signed short)(targetDegrees*(META_scaleFactor/360));
}

bool MATH_withinThreshold(float inNumber, float max, float min){
	return (inNumber < max && inNumber > min);
}

//PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----//

byte MATH_calculatePID(TEMPLATE_PID &values, signed int target, signed int sensorInput){

	//Calculate Error
	values.error = (signed int)(target - sensorInput);

	//Calculate integral if within the integralMax range
	if(fabs(values.integral+values.error * META_loopsDelay * 0.01)<= (float)(values.integralMax)){
		values.integral += values.error * META_loopsDelay * 0.01;
	}
	if(MATH_withinThreshold(values.error, values.correctionThreshold, -values.correctionThreshold)) values.integral = 0;

	//Calculate derivative and output
	values.output = MATH_clamp((values.KP * values.error) +
	(values.KI * values.integral) +
	(values.KD * ((values.error - values.lastError)/(META_loopsDelay * 0.001))));

	//Assign error to last error
	values.lastError = values.error;

	//Check if values.output has remained within the correctionThreshold for a values.correctionCycles amount of cycles
	if(MATH_withinThreshold(values.output, values.correctionThreshold, -values.correctionThreshold)){
		if(values.cyclesCounter<(ubyte)values.correctionCycles) values.cyclesCounter++;
		writeDebugStream("Inside Threshold\t");
		if(values.cyclesCounter == (ubyte)values.correctionCycles){
			if(MATH_withinThreshold(values.output, values.correctionThreshold, -values.correctionThreshold)){
				values.notDone = false;
				values.output = 0;
				return 0;
			}
			else values.notDone = true;
		}
	}
	else values.cyclesCounter = 0;

	//Timeout so PID doesn't take too long
	values.timeoutCounter++;
	if(values.timeoutCounter >= values.timeout){
		writeDebugStreamLine("\nTimeout!!");
		values.notDone = false;
		values.output = 0;
		return 0;
	}
	//Just in case Debug is needed
	//writeDebugStreamLine("values = %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", values.KP, values.KI, values.KD, values.integralMax, values.error, values.lastError, values.integral, values.cyclesCounter, values.timeoutCounter, values.output);
	datalogAddValue(0, sensorInput);

	return (values.output);

}

double MATH_motionProfile(TEMPLATE_motionProfile &values, signed int currentDistance, unsigned int desiredDistance, signed int desiredSpeed){

	if(0<=currentDistance && currentDistance<desiredDistance*values.distanceMultiplier[0]){
		return( ((desiredSpeed-values.offset)/(desiredDistance*values.distanceMultiplier[0]))*currentDistance + values.offset );
	}

	else if(desiredDistance*values.distanceMultiplier[0]<=currentDistance && currentDistance<desiredDistance*values.distanceMultiplier[1]){
		return( desiredSpeed );
	}

	else if(desiredDistance*values.distanceMultiplier[1] <= currentDistance && (unsigned int)(currentDistance) <= desiredDistance){
		return( ((-desiredSpeed)/(desiredDistance-desiredDistance*values.distanceMultiplier[1])) * (currentDistance - desiredDistance*values.distanceMultiplier[1])+desiredSpeed );
	}

	return 0;
}

#endif
