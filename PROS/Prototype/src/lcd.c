#include "main.h"

void lcdDisplay() {
	switch (currentMenu) {
	case lcdMenus::batteryVoltage:
		lcdClear(LCD_PORT);

		*lcdOutput = "<    %1.3f     >", ((float)powerLevelMain() / 1000);
		lcdSetText(LCD_PORT, 1, "Battery Voltage");
		lcdSetText(LCD_PORT, 2, *lcdOutput);
		break;

	case lcdMenus::backupBatteryVoltage:
		lcdClear(LCD_PORT);

		*lcdOutput = "<    %1.3f     >", ((float)powerLevelBackup() / 1000);
		lcdSetText(LCD_PORT, 1, "Backup Battery V");
		lcdSetText(LCD_PORT, 2, *lcdOutput);
		break;

	case lcdMenus::autonomous:
		if (currentCode == autonomousCodes::mogoAndCones)	*lcdOutput = "< MoGo + Cones >";
		else if (currentCode == autonomousCodes::cones)	*lcdOutput = "<     Cones    >";
		else if (currentCode == autonomousCodes::mogo)	*lcdOutput = "<  Mobile Goal >";

		lcdClear(LCD_PORT);

		lcdSetText(LCD_PORT, 1, "  Current Auton ");
		lcdSetText(LCD_PORT, 2, *lcdOutput);
		break;

	case lcdMenus::allianceColor:
		if (currentColor == allianceColor::red)	*lcdOutput = "<     Red      >";
		else if (currentColor == allianceColor::blue)	*lcdOutput = "<     Blue     >";

		lcdClear(LCD_PORT);

		lcdSetText(LCD_PORT, 1, "  Current Auton ");
		lcdSetText(LCD_PORT, 2, *lcdOutput);
		break;

	case lcdMenus::startingSide:
		if (currentSide == startingSide::right)	*lcdOutput = "<     Right    >";
		else if (currentSide == startingSide::left)	*lcdOutput = "<     Left     >";

		lcdClear(LCD_PORT);

		lcdSetText(LCD_PORT, 1, "  Current Side  ");
		lcdSetText(LCD_PORT, 2, *lcdOutput);
		break;

	default:
		lcdClear(LCD_PORT);
		lcdSetText(LCD_PORT, 1, "     2223-Z     ");
		lcdSetText(LCD_PORT, 2, "<      OK      >");
		break;
	}
}

void lcdSelect() {
	if (lcdReadButtons(LCD_PORT) == 1) {
		if (!lcdButtonsPressed[2]) {
			lcdButtonsPressed[2] = true;
			switch(currentMenu){
				case lcdMenus::main:
					currentMenu = lcdMenus::batteryVoltage;
				break;
				case lcdMenus::batteryVoltage:
				currentMenu = lcdMenus::backupBatteryVoltage;
				break;
				case lcdMenus::backupBatteryVoltage:
				currentMenu = lcdMenus::autonomous;
				break;
				case lcdMenus::autonomous:
					currentMenu = lcdMenus::allianceColor;
				break;
				case lcdMenus::allianceColor:
				currentMenu = lcdMenus::startingSide;
				break;
				case lcdMenus::startingSide:
					currentMenu = lcdMenus::main;
				break;
			}
			lcdDisplay();
		}
	}
	else lcdButtonsPressed[2] = false;

	if (lcdReadButtons(LCD_PORT) == 4) {
		if (!lcdButtonsPressed[0]) {
			lcdButtonsPressed[2] = true;
			switch(currentMenu){
				case lcdMenus::main:
				currentMenu = lcdMenus::startingSide;
				break;
				case lcdMenus::startingSide:
				currentMenu = lcdMenus::allianceColor;
				break;
				case lcdMenus::allianceColor:
				currentMenu = lcdMenus::autonomous;
				break;
				case lcdMenus::autonomous:
				currentMenu = lcdMenus::backupBatteryVoltage;
				break;
				case lcdMenus::backupBatteryVoltage:
				currentMenu = lcdMenus::batteryVoltage;
				break;
				case lcdMenus::batteryVoltage:
				currentMenu = lcdMenus::main;
				break;
			}
			lcdDisplay();
		}
	}
	else lcdButtonsPressed[0] = false;

	if (lcdReadButtons(LCD_PORT) == 2) {
		if (!lcdButtonsPressed[1]) {
			lcdButtonsPressed[1] = true;
			switch (currentMenu) {
			case lcdMenus::main:
				lcdReady = true;
				break;
			case lcdMenus::batteryVoltage:
				lcdDisplay();
				break;
			case lcdMenus::backupBatteryVoltage:
				lcdDisplay();
				break;
			case lcdMenus::autonomous:
				currentCode = (currentCode == autonomousCodes::cones) ? autonomousCodes::mogoAndCones :
					(currentCode == autonomousCodes::mogoAndCones) ? autonomousCodes::mogo :
					autonomousCodes::cones;
				lcdDisplay();
				break;
			case lcdMenus::allianceColor:
				currentColor = currentColor == allianceColor::blue ? allianceColor::red : allianceColor::blue;
				lcdDisplay();
				break;
			case lcdMenus::startingSide:
				currentSide = currentSide == startingSide::left ? startingSide::right : startingSide::left;
				lcdDisplay();
				break;
			}
		}
	}
	else lcdButtonsPressed[1] = false;

}


//http://www.codeguru.com/cpp/cpp/article.php/c19083/C-2011-Stronglytyped-Enums.htm
