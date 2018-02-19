/*   Draft of possible formulas to use as tools in the future               *
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

#pragma config(Sensor, in1,    armPot,         sensorPotentiometer)
#pragma config(Sensor, dgtl1,  LeftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  RightEncoder,   sensorQuadEncoder)

typedef struct{
	float baseWidth;	float wheelDiameter;	unsigned short rotationTicks;	//You HAVE to initialize this
	unsigned short pulses;	//Traslation
	short grados;	unsigned short ticks;	//Rotation
} robotBase;

typedef struct{
	short maxPotValue;	//You HAVE to initialize this
	short position;	short grados;	short ticks;	//Arm control
} robotArm;

task main(){
	/*Initialization*/
	robotBase baseLeft; robotBase baseRight; robotArm armLeft; robotArm armRight;
	baseLeft.baseWidth = 16.5;	baseRight.baseWidth = 16.5;	baseLeft.wheelDiameter = 3.25;	baseRight.wheelDiameter = 3.25;
	baseLeft.rotationTicks = 4000;	baseRight.rotationTicks = 4000;	armLeft.maxPotValue = 1024;	armRight.maxPotValue = 1024;

	SensorValue[LeftEncoder] = 0; SensorValue[RightEncoder] = 0;
	/*Initialization*/

	baseLeft.grados = 90;
	baseLeft.pulses = floor( ((baseLeft.grados/360) * baseLeft.baseWidth * 3.1415926535897932384626433832795) / (baseLeft.wheelDiameter / 360) );

	//Pulses needed to rotate the base a total of 'degrees' degrees with Gyroscope
	baseLeft.ticks = ceil(baseLeft.grados *(baseLeft.rotationTicks/360));

	//Pulses needed to rotate the arm a total of 'degrees' degrees with Potentiometers
	armLeft.grados = -90;
	armLeft.ticks = armLeft.position + (floor(armLeft.grados * (armLeft.maxPotValue / 270)));
}
