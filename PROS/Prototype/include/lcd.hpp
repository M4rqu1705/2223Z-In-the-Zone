/*#ifndef LCD_HPP_
#define LCD_HPP_

#include <main.hpp>
#include <string>

#if USING_LCD
namespace LCD {
	toggleButton buttonsPressed[3] = { false, false, false };

	//Create enumerated type variable lcdMenus to indicate the available menus
	enum menus : unint_fast8_t { mainMenu = 0, batteryVoltageMenu, backupBatteryVoltageMenu, powerExpanderMenu, autonomousMenu, allianceColorMenu, startingSideMenu };
	menus currentMenu = mainMenu;
	//Create enumerated type variable autonomousCodes to indicate the things the robot can do (which will then be used to determine which autonomous to execute)
	enum codes : unint_fast8_t { mogoAndCones = 0, mogo, cones };
	codes currentCode = mogoAndCones;
	//Create enumerated type variable allianceColor to indicate which is the color of the robot's alliance (which will then be used to determine which autonomous to execute)
	enum color : unint_fast8_t { red = 0, blue };
	color currentColor = red;
	//Create enumerated type variable startingSide to indicate which side will the autonomous be executed (which will then be used to determine which autonomous to execute)
	enum startingSide : unint_fast8_t { left = 0, right };
	startingSide currentSide = left;

	std::string output;    //Declare variable in which the formatted string containing a variable LCD output will be stored
	bool notDone;       //Declare a boolean to indicate if autonomous has been selected or not

	void select();    //Function prototype for the lcdSelect() function which will be declared at the end because of its length

}
#endif
#endif
*/
