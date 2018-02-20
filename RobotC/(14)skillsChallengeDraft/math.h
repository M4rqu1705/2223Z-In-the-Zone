/*   math.h - Container of useful mathematical functions                    *
*    Copyright (C) <2018>  Marcos Ricardo Pesante Colón                     *
*                                                                           *
*    This program is free software: you can redistribute it and/or modify   *
*    it under the terms of the GNU General Public License as published by   *
*    the Free Software Foundation, either version 3 of the License, or      *
*    (at your option) any later version.                                    *
*                                                                           *
*    This program is distributed in the hope that it will be useful,        *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of         *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
*    GNU General Public License for more details.                           *
*                                                                           *
*    You should have received a copy of the GNU General Public License      *
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#ifndef MATH_H_
#define MATH_H_


#pragma systemfile

int MATH_map(int inNumber, int inMax, int inMin, int outMax, int outMin){
	return  (inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
	//Retrieved from https://www.arduino.cc/reference/en/language/functions/math/map/
}

byte MATH_clamp( int inNumber){
	if(inNumber>127) return 127;
	else if (inNumber<-127) return -127;
	else return ( byte)(inNumber);
}

int MATH_round(float inNumber){
	if(inNumber>=0) return (( int)(ceil(inNumber-0.49)));
	else return (( int)(floor(inNumber+0.49)));
}

short MATH_inchesToPulses(float inches){
	return MATH_round(inches*(360/(META_driveWheelDiameter*PI)));
}

short MATH_degreesToPulses(float targetDegrees, float turnRadius){
	return MATH_inchesToPulses((targetDegrees*PI*turnRadius)/180);
	//return (angle*PI*r)/180
}

float MATH_swingTurnInside(float turnRadius, float speed){
	return (-(META_driveWidth/turnRadius) * speed + speed);
}

float MATH_degreesToTicks(float targetDegrees){
	return (float)(targetDegrees*10);
}

bool MATH_withinThreshold(float inNumber, float max, float min){
	return (inNumber < max && inNumber > min);
}

float MATH_getSpeed(float &previousValue, float currentValue){
	float temp = (currentValue-previousValue)/(META_loopsDelay*0.001);
	previousValue = currentValue;
	return temp;
}

//PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----//

byte MATH_calculatePID(TEMPLATE_PID &values,  int setPoint,  int processVariable){

	//Calculate Error
	values.error = ( int)(setPoint - processVariable);

	//Calculate integral if within the integralMax range
	if(fabs(values.integral+values.error * META_loopsDelay * 0.001)<= (float)(values.integralMax)){
		values.integral += values.error * META_loopsDelay * 0.001;
	}
	if(MATH_withinThreshold(values.error, values.correctionThreshold, -values.correctionThreshold)) values.integral = 0;

	//Calculate derivative and output
	values.output = MATH_clamp((values.KP * values.error) +
	(values.KI * values.integral) +
	(values.KD * ((values.error - values.lastError)/(META_loopsDelay * 0.001))));

	//Assign error to last error
	values.lastError = values.error;

	//Check if values.output has remained within the correctionThreshold for a values.correctionCycles amount of cycles
	if(MATH_withinThreshold(processVariable, (setPoint + values.correctionThreshold), (setPoint - values.correctionThreshold))){
		if(values.cyclesCounter<(ubyte)values.correctionCycles) values.cyclesCounter++;
		writeDebugStream("Inside Threshold\t");
		if(values.cyclesCounter == (ubyte)values.correctionCycles){
			if(MATH_withinThreshold(processVariable, (setPoint + values.correctionThreshold), (setPoint - values.correctionThreshold))){
				values.notDone = false;
				values.output = 0;
				return 0;
			}
			else values.notDone = true;
		}
	}
	else values.cyclesCounter = 0;

	//Just in case Debug is needed
	//writeDebugStreamLine("values = %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", values.KP, values.KI, values.KD, values.integralMax, values.error, values.lastError, values.integral, values.cyclesCounter, values.timeoutCounter, values.output); datalogAddValue(0, processVariable);

	return (values.output);

}

float MATH_motionProfile(TEMPLATE_motionProfile &values,  int currentDistance,  int desiredDistance,  int desiredSpeed){
	//Output value from generated motion profile based on current distance as an 'x' variable

	if(0<=currentDistance && currentDistance<desiredDistance*values.distanceMultiplier[0]){
		return( ((desiredSpeed-values.offsets[0])/(desiredDistance*values.distanceMultiplier[0]))*currentDistance + values.offsets[0] );
	}

	else if(desiredDistance*values.distanceMultiplier[0]<=currentDistance && currentDistance<desiredDistance*values.distanceMultiplier[1]){
		return( desiredSpeed );
	}

	else if(desiredDistance*values.distanceMultiplier[1] <= currentDistance && ( int)(currentDistance) <= desiredDistance){
		return( ((-desiredSpeed-values.offsets[1])/(desiredDistance-desiredDistance*values.distanceMultiplier[1])) * (currentDistance - desiredDistance*values.distanceMultiplier[1])+desiredSpeed );
	}

	return 0;
}

#endif
