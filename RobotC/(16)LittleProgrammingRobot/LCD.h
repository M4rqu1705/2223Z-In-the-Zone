/*   LCD.h - "Driver" for LCD autonomous selection                          *
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

#ifndef LCD.h
#define LCD.h

#pragma systemfile

//Create enumerated type variable lcdMenus to indicate the available menus
enum ENUM_lcdMenus { mainMenu = 0, batteryVoltageMenu, backupBatteryVoltageMenu, mobileGoalScoreMenu, coneScoreMenu, startingSideMenu};
ENUM_lcdMenus currentMenu = mainMenu;
//Create byte array to contain how the autonomous might score Mobile Goals or cones (format: {mobileGoalZone, amountOfCones}
byte elementsToScore[2] = {0, 0};
//Create boolean currentSideRight to indicate which side will the autonomous be executed (which will then be used to determine which autonomous to execute)
bool currentSideRight = true;

bool lcdButtonsPressed[3];
string lcdOutput;    //Declare variable in which the formatted string containing a variable LCD output will be stored

void LCD_init(){

	configureSerialPort(UART0, uartNotUsed);
	setBaudRate(UART0, baudRate4800);
	configureSerialPort(UART1, uartVEXLCD);
	setBaudRate(UART1, baudRate19200);
	configureSerialPort(UART2, uartNotUsed);
	setBaudRate(UART2, baudRate4800);

	clearLCDLine(0);                  //Clear LCD
	clearLCDLine(1);                  //Clear LCD
	bLCDBacklight = true;    //Turn backlight

	currentMenu = mainMenu;        //Preset to main menu
	elementsToScore[0] = 20;	elementsToScore[1] = 3;
	currentSideRight = true;       //Preset autonomous starting side to right side

	//None of the LCD buttons have been previously pressed
	lcdButtonsPressed[0] = false;
	lcdButtonsPressed[1] = false;
	lcdButtonsPressed[2] = false;

	bLCDBacklight = true;

	/* Copy and paste at the end of Initialization

	while(bIfiRobotDisabled){
	lcdSelect();
	}

	*/
}

void LCD_select(){
	if (nLCDButtons == 4) { //Right button
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
				currentMenu = mobileGoalScoreMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				sprintf(lcdOutput, "< %2%d >    %2%d", elementsToScore[0], elementsToScore[1]);

				displayLCDCenteredString(0, "MobileGoal Score");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case mobileGoalScoreMenu:
				currentMenu = coneScoreMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "Cone Amount");
				sprintf(lcdOutput, " %2%d    < %2%d >", elementsToScore[0], elementsToScore[1]);
				break;

			case coneScoreMenu:
				currentMenu = startingSideMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSideRight == false)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSideRight == true)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Starting Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentMenu = mainMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				displayLCDCenteredString(1, "<  Change Menu >");
				break;
			}
		}
	}
	else lcdButtonsPressed[2] = false;

	if (nLCDButtons == 1) {		//Left button
		if (!lcdButtonsPressed[0]) {
			lcdButtonsPressed[0] = true;
			switch (currentMenu) {

			case mainMenu:
				currentMenu = startingSideMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSideRight == false)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSideRight == true)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Starting Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case batteryVoltageMenu:
				currentMenu = mainMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				displayLCDCenteredString(1, "<  Change Menu >");
				break;

			case backupBatteryVoltageMenu:
				currentMenu = batteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Battery Voltage");
				sprintf(lcdOutput, "<     %1.2f%s", nImmediateBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case mobileGoalScoreMenu:
				currentMenu = backupBatteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Backup Battery V");
				sprintf(lcdOutput, "<     %1.2f%s", BackupBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case coneScoreMenu:
				currentMenu = mobileGoalScoreMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				sprintf(lcdOutput, "< %2%d >    %2%d", elementsToScore[0], elementsToScore[1]);

				displayLCDCenteredString(0, "MobileGoal Score");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentMenu = coneScoreMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "Cone Amount");
				sprintf(lcdOutput, " %2%d    < %2%d >", elementsToScore[0], elementsToScore[1]);
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

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				displayLCDCenteredString(1, "<  Change Menu >");

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


			case mobileGoalScoreMenu:
				if(elementsToScore[0] == 0) elementsToScore[0] = 5;
				else if(elementsToScore[0] == 5) elementsToScore[0] = 10;
				else if(elementsToScore[0] == 10) elementsToScore[0] = 20;
				else if(elementsToScore[0] == 20) elementsToScore[0] = 0;

				clearLCDLine(0);
				clearLCDLine(1);

				sprintf(lcdOutput, "< %2%d >    %2%d", elementsToScore[0], elementsToScore[1]);

				displayLCDCenteredString(0, "MobileGoal Score");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case coneScoreMenu:
				if(elementsToScore[1] == 1) elementsToScore[1] = 2;
				else if(elementsToScore[1] == 2) elementsToScore[1] = 3;
				else if(elementsToScore[1] == 3) elementsToScore[1] = 1;

				clearLCDLine(0);
				clearLCDLine(1);

				sprintf(lcdOutput, "%2%d    < %2%d >", elementsToScore[0], elementsToScore[1]);

				displayLCDCenteredString(0, "Cone Amount");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentSideRight = !currentSideRight;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSideRight == false)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSideRight == true)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			}
		}
	}
	else lcdButtonsPressed[1] = false;
}
#endif
