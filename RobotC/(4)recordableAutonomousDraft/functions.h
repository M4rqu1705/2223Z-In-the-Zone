/*   functions.h - Functions for the recordable autonomous functions        *
*    Copyright (C) <2017>  Marcos Ricardo Pesante Colón                     *
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
