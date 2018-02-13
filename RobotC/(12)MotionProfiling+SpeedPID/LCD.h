#ifndef LCD.h
#define LCD.h

#pragma systemfile

#ifdef META_usingLCD    //Only include code if META_usingLCD is defined in the constants.h file
//Create enumerated type variable lcdMenus to indicate the available menus
enum lcdMenus { mainMenu = 0, batteryVoltageMenu, backupBatteryVoltageMenu, powerExpanderMenu, autonomousMenu, allianceColorMenu, startingSideMenu};
lcdMenus currentMenu = mainMenu;
//Create enumerated type variable autonomousCodes to indicate the things the robot can do (which will then be used to determine which autonomous to execute)
enum autonomousCodes { fivePointMogo=0, tenPointMogo, twentyPointMogo, programmingSkillsCode };
autonomousCodes currentCode = twentyPointMogo;
//Create enumerated type variable allianceColor to indicate which is the color of the robot's alliance (which will then be used to determine which autonomous to execute)
bool currentColorRed = true ;
//Create enumerated type variable startingSide to indicate which side will the autonomous be executed (which will then be used to determine which autonomous to execute)
bool currentSideRight = true;

bool lcdButtonsPressed[3];
string lcdOutput;    //Declare variable in which the formatted string containing a variable LCD output will be stored
bool lcdReady;       //Declare a boolean to indicate if autonomous has been selected or not

void LCD_init(){
	clearLCDLine(0);                  //Clear LCD
	clearLCDLine(1);                  //Clear LCD
	bLCDBacklight = META_LCDbacklight;    //Turn backlight on or off based on META_LCDbacklight constant

	currentMenu = mainMenu;        //Preset to main menu
	currentCode = programmingSkillsCode;    //Preset autonomous to Mobile Goal and Cones
	currentColorRed = true;            //Preset autonomous alliance color to red
	currentSideRight = true;       //Preset autonomous starting side to right side

	//None of the LCD buttons have been previously pressed
	lcdButtonsPressed[0] = false;
	lcdButtonsPressed[1] = false;
	lcdButtonsPressed[2] = false;

	if (bIfiRobotDisabled){
		lcdReady = false;
	}
	else {                     //If robot is starting up again enabled (maybe it disconnected from the field) don't calibrate gyro and make robot just start again
		lcdReady = true;
	}
	bLCDBacklight = META_lcdBacklight;

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

void LCD_select(){
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
				sprintf(lcdOutput, "<     %1.2f%s", SensorValue[SENSOR_powerExpanderStatus]/META_powerExpanderInputDivisor,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case powerExpanderMenu:
				currentMenu = autonomousMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == twentyPointMogo)	sprintf(lcdOutput, "< 20 PointMogo >");
				else if (currentCode == tenPointMogo)	sprintf(lcdOutput, "< 10 PointMogo >");
				else if (currentCode == fivePointMogo)	sprintf(lcdOutput, "< 5 Point Mogo >");
				else if (currentCode == programmingSkillsCode) sprintf(lcdOutput, "< Progr Skills >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentMenu = allianceColorMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColorRed == true)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColorRed == false)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentMenu = startingSideMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSideRight == false)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSideRight == true)	sprintf(lcdOutput, "<     Right    >");

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

				if (currentSideRight == false)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSideRight == true)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentMenu = allianceColorMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColorRed == true)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColorRed == false)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentMenu = autonomousMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == twentyPointMogo)	sprintf(lcdOutput, "< 20Point Mogo >");
				else if (currentCode == tenPointMogo)	sprintf(lcdOutput, "< 10Point Mogo >");
				else if (currentCode == fivePointMogo)	sprintf(lcdOutput, "< 5 Point Mogo >");
				else if (currentCode == programmingSkillsCode) sprintf(lcdOutput, "< Progr Skills >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentMenu = powerExpanderMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Power Expander V");
				sprintf(lcdOutput, "<     %1.2f%s", SensorValue[SENSOR_powerExpanderStatus]/META_powerExpanderInputDivisor,"V    >");
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
				sprintf(lcdOutput, "<     %1.2f%s", SensorValue[SENSOR_powerExpanderStatus]/META_powerExpanderInputDivisor,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentCode = (currentCode == twentyPointMogo) ? tenPointMogo :
				(currentCode == tenPointMogo) ? fivePointMogo : (currentCode == fivePointMogo) ? programmingSkillsCode :
				twentyPointMogo;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == twentyPointMogo)	sprintf(lcdOutput, "< 20Point Mogo >");
				else if (currentCode == tenPointMogo)	sprintf(lcdOutput, "< 10Point Mogo >");
				else if (currentCode == fivePointMogo)	sprintf(lcdOutput, "< 5 Point Mogo >");
				else if (currentCode == programmingSkillsCode) sprintf(lcdOutput, "< Progr Skills >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentColorRed = (currentColorRed == false) ? true : false;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColorRed == true)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColorRed == false)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentSideRight = (currentSideRight == false) ? true : false;

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
#endif
