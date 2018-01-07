#ifndef fullRobotTestLCD.h
#define fullRobotTestLCD.h

#pragma systemfile

#ifdef META_usingLCD    //Only include code if META_usingLCD is defined in the constants.h file
enum PIDconstantsEnum {mogoIntakeSubsExtendKP, mogoIntakeSubsExtendKI, mogoIntakeSubsExtendKD,
	mogoIntakeSubsRetractKP, mogoIntakeSubsRetractKI, mogoIntakeSubsRetractKD,
	armSubsKP, armSubsKI, armSubsKD};

PIDconstantsEnum lcdCurrentConstant = mogoIntakeSubsExtendKP;

bool lcdButtonsPressed[3];
string lcdOutput;    //Declare variable in which the formatted string containing a variable LCD output will be stored

void lcdInit(){
	clearLCDLine(0);                  //Clear LCD
	clearLCDLine(1);                  //Clear LCD
	bLCDBacklight = META_LCDbacklight;    //Turn backlight on or off based on META_LCDbacklight constant

	PIDconstantsEnum lcdCurrentConstant = mogoIntakeSubsExtendKP;
	displayLCDCenteredString(0, "MogoRetractKP");
	sprintf(lcdOutput, "%1.2f", PID_KPmogoRetract);
	displayLCDCenteredString(1, lcdOutput);

	//None of the LCD buttons have been previously pressed
	lcdButtonsPressed[0] = false;
	lcdButtonsPressed[1] = false;
	lcdButtonsPressed[2] = false;

}

void lcdCalibrate(){		//It is in reality used for calibrating PID
	switch(nLCDButtons){
	case (TControllerButtons)2:
		if(!lcdButtonsPressed[1]){
			lcdButtonsPressed[1] = true;
			switch(lcdCurrentConstant){
			case mogoIntakeSubsExtendKP:
				lcdCurrentConstant = mogoIntakeSubsExtendKI;
				displayLCDCenteredString(0, "MogoExtendKI");
				sprintf(lcdOutput, "%1.2f", PID_KImogoExtend);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsExtendKI:
				lcdCurrentConstant = mogoIntakeSubsExtendKD;
				displayLCDCenteredString(0, "MogoExtendKD");
				sprintf(lcdOutput, "%1.2f", PID_KDmogoExtend);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsExtendKD:
				lcdCurrentConstant = mogoIntakeSubsRetractKP;
				displayLCDCenteredString(0, "MogoRetractKP");
				sprintf(lcdOutput, "%1.2f", PID_KPmogoRetract);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsRetractKP:
				lcdCurrentConstant = mogoIntakeSubsRetractKI;
				displayLCDCenteredString(0, "MogoRetractKI");
				sprintf(lcdOutput, "%1.2f", PID_KImogoRetract);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsRetractKI:
				lcdCurrentConstant = mogoIntakeSubsRetractKD;
				displayLCDCenteredString(0, "MogoRetractKD");
				sprintf(lcdOutput, "%1.2f", PID_KDmogoRetract);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsRetractKD:
				lcdCurrentConstant = armSubsKP;
				displayLCDCenteredString(0, "Arm KP");
				sprintf(lcdOutput, "%1.2f", PID_KParm);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case armSubsKP:
				lcdCurrentConstant = armSubsKI;
				displayLCDCenteredString(0, "Arm KI");
				sprintf(lcdOutput, "%1.2f", PID_KIarm);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case armSubsKI:
				lcdCurrentConstant = armSubsKD;
				displayLCDCenteredString(0, "Arm KD");
				sprintf(lcdOutput, "%1.2f", PID_KDarm);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case armSubsKD:
				lcdCurrentConstant = mogoIntakeSubsExtendKP;
				displayLCDCenteredString(0, "MogoExtendKP");
				sprintf(lcdOutput, "%1.2f", PID_KPmogoExtend);
				displayLCDCenteredString(1, lcdOutput);
				break;
			}
		}
		break;
	case (TControllerButtons)1:	//If left button pressed
		if(!lcdButtonsPressed[0]){
			lcdButtonsPressed[0] = true;
			switch(lcdCurrentConstant){
			case mogoIntakeSubsExtendKP:
				PID_KPmogoExtend -= 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KPmogoExtend);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsExtendKI:
				PID_KImogoExtend -= 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KImogoExtend);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsExtendKD:
				PID_KDmogoExtend -= 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KDmogoExtend);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsRetractKP:
				PID_KPmogoRetract -= 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KPmogoRetract);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsRetractKI:
				PID_KImogoRetract -= 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KImogoRetract);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsRetractKD:
				PID_KDmogoRetract -= 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KDmogoRetract);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case armSubsKP:
				PID_KParm -= 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KParm);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case armSubsKI:
				PID_KIarm -= 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KIarm);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case armSubsKD:
				PID_KDarm -= 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KDarm);
				displayLCDCenteredString(1, lcdOutput);
				break;
			}
		}
		break;

	case (TControllerButtons)4:	//if right button pressed
		if(!lcdButtonsPressed[2]){
			lcdButtonsPressed[2] = true;
			switch(lcdCurrentConstant){
			case mogoIntakeSubsExtendKP:
				PID_KPmogoExtend += 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KPmogoExtend);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsExtendKI:
				PID_KImogoExtend += 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KImogoExtend);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsExtendKD:
				PID_KDmogoExtend += 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KDmogoExtend);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsRetractKP:
				PID_KPmogoRetract += 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KPmogoRetract);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsRetractKI:
				PID_KImogoRetract += 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KImogoRetract);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case mogoIntakeSubsRetractKD:
				PID_KDmogoRetract += 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KDmogoRetract);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case armSubsKP:
				PID_KParm += 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KParm);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case armSubsKI:
				PID_KIarm += 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KIarm);
				displayLCDCenteredString(1, lcdOutput);
				break;
			case armSubsKD:
				PID_KDarm += 0.05;
				sprintf(lcdOutput, "%1.2f", PID_KDarm);
				displayLCDCenteredString(1, lcdOutput);
				break;
			}
		}
		break;
	default:
		lcdButtonsPressed[0] = false;
		lcdButtonsPressed[1] = false;
		lcdButtonsPressed[2] = false;
	}
}
#endif
#endif
