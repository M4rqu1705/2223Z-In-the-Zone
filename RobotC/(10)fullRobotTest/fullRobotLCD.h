/*   fullRobotLCD.h - Autonomous selection program                          *
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

#ifndef fullRobotTestLCD.h
#define fullRobotTestLCD.h

#pragma systemfile

#ifdef META_usingLCD    //Only include code if META_usingLCD is defined in the constants.h file
//Create enumerated type variable lcdMenus to indicate the available menus
enum lcdMenus { mainMenu = 0, batteryVoltageMenu, backupBatteryVoltageMenu, powerExpanderMenu, autonomousMenu, allianceColorMenu, startingSideMenu};
lcdMenus currentMenu = mainMenu;
//Create enumerated type variable autonomousCodes to indicate the things the robot can do (which will then be used to determine which autonomous to execute)
enum autonomousCodes { mogoAndCones = 0, mogo, cones };
autonomousCodes currentCode = mogoAndCones;
//Create enumerated type variable allianceColor to indicate which is the color of the robot's alliance (which will then be used to determine which autonomous to execute)
enum allianceColor { red = 0, blue};
allianceColor currentColor = red;
//Create enumerated type variable startingSide to indicate which side will the autonomous be executed (which will then be used to determine which autonomous to execute)
enum startingSide { leftSide = 0, rightSide};
startingSide currentSide = rightSide;

string lcdOutput;    //Declare variable in which the formatted string containing a variable LCD output will be stored
bool lcdReady;       //Declare a boolean to indicate if autonomous has been selected or not

void lcdInit(){
	clearLCDLine(0);                  //Clear LCD
	clearLCDLine(1);                  //Clear LCD
	bLCDBacklight = META_LCDbacklight;    //Turn backlight on or off based on META_LCDbacklight constant

	currentMenu = mainMenu;        //Preset to main menu
	currentCode = mogoAndCones;    //Preset autonomous to Mobile Goal and Cones
	currentColor = red;            //Preset autonomous alliance color to red
	currentSide = rightSide;       //Preset autonomous starting side to right side

	//None of the LCD buttons have been previously pressed
	lcdButtonsPressed[0] = false;
	lcdButtonsPressed[1] = false;
	lcdButtonsPressed[2] = false;

	if (bIfiRobotDisabled){    //Display that Gyro is calibrating only if robot is disabled
		lcdReady = false;
		displayLCDCenteredString(0, "Calibrating Gyro");
		displayLCDString(1, 0, "...");
	}
	else {                     //If robot is starting up again enabled (maybe it disconnected from the field) don't calibrate gyro and make robot just start again
		lcdReady = true;
	}
	bLCDBacklight = false;

	/* Copy and paste at the end of Initialization

	//Only include piece of code if META_usingLCD is defined
#ifdef META_usingLCD
	do{
	if(bIfiRobotDisabled)	lcdSelect();
	else break;
	}while (!lcdReady);

	//Clear the LCD
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDCenteredString(0, "     2223-Z     ");    //Output 2223-Z on the screen, signaling that the lcd is done
#endif

	*/

}

void lcdSelect(){
	if (nLCDButtons == 4) {
		if (!lcdButtonsPressed[2]) {
			lcdButtonsPressed[2] = true;
			switch (currentMenu) {
			case mainMenu:
				currentMenu = batteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Battery Voltage");
				sprintf(lcdOutput, "<     %1.2f%s", nImmediateBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case batteryVoltageMenu:
				currentMenu = backupBatteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Backup Battery V");
				sprintf(lcdOutput, "<     %1.2f%s", BackupBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case backupBatteryVoltageMenu:
				currentMenu = powerExpanderMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Power Expander V");
				sprintf(lcdOutput, "<     %1.2f%s", SensorValue[POWER_EXPANDER_STATUS]/POWER_EXPANDER_DIVISOR,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case powerExpanderMenu:
				currentMenu = autonomousMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == mogoAndCones)	sprintf(lcdOutput, "< MoGo + Cones >");
				else if (currentCode == cones)	sprintf(lcdOutput, "<     Cones    >");
				else if (currentCode == mogo)	sprintf(lcdOutput, "<  Mobile Goal >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentMenu = allianceColorMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColor == red)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColor == blue)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentMenu = startingSideMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSide == leftSide)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSide == rightSide)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentMenu = mainMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				displayLCDCenteredString(1, "<      OK      >");
				break;
			}
		}
	}
	else lcdButtonsPressed[2] = false;

	if (nLCDButtons == 1) {
		if (!lcdButtonsPressed[0]) {
			lcdButtonsPressed[0] = true;
			switch (currentMenu) {
			case mainMenu:
				currentMenu = startingSideMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSide == leftSide)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSide == rightSide)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentMenu = allianceColorMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColor == red)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColor == blue)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentMenu = autonomousMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == mogoAndCones)	sprintf(lcdOutput, "< MoGo + Cones >");
				else if (currentCode == cones)	sprintf(lcdOutput, "<     Cones    >");
				else if (currentCode == mogo)	sprintf(lcdOutput, "<  Mobile Goal >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentMenu = powerExpanderMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Power Expander V");
				sprintf(lcdOutput, "<     %1.2f%s", SensorValue[POWER_EXPANDER_STATUS]/POWER_EXPANDER_DIVISOR,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case powerExpanderMenu:
				currentMenu = backupBatteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Backup Battery V");
				sprintf(lcdOutput, "<     %1.2f%s", BackupBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case backupBatteryVoltageMenu:
				currentMenu = batteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Battery Voltage");
				sprintf(lcdOutput, "<     %1.2f%s", nImmediateBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case batteryVoltageMenu:
				currentMenu = mainMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				displayLCDCenteredString(1, "<      OK      >");
				break;
			}
		}
	}
	else lcdButtonsPressed[0] = false;

	if (nLCDButtons == 2) {
		if (!lcdButtonsPressed[1]) {
			lcdButtonsPressed[1] = true;
			switch (currentMenu) {
			case mainMenu:
				lcdReady = true;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				break;

			case batteryVoltageMenu:
				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Battery Voltage");
				sprintf(lcdOutput, "<     %1.2f%s", nImmediateBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;
			case backupBatteryVoltageMenu:
				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Backup Battery V");
				sprintf(lcdOutput, "<     %1.2f%s", BackupBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case powerExpanderMenu:
				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Power Expander V");
				sprintf(lcdOutput, "<     %1.2f%s", SensorValue[POWER_EXPANDER_STATUS]/POWER_EXPANDER_DIVISOR,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentCode = (currentCode == cones) ? mogoAndCones :
				(currentCode == mogoAndCones) ? mogo :
				cones;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == mogoAndCones)	sprintf(lcdOutput, "< MoGo + Cones >");
				else if (currentCode == cones)	sprintf(lcdOutput, "<     Cones    >");
				else if (currentCode == mogo)	sprintf(lcdOutput, "<  Mobile Goal >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentColor = currentColor == blue ? red : blue;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColor == red)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColor == blue)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentSide = currentSide == leftSide ? rightSide : leftSide;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSide == leftSide)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSide == rightSide)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			}
		}
	}
	else lcdButtonsPressed[1] = false;
}
#endif
#endif
