#pragma config(Motor,	port1,	DriveBR,	tmotorVex393HighSpeed_HBridge, openLoop, reversed)
#pragma config(Motor,	port2,	DriveFR,	tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,	port9,	DriveFL,	tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,	port10,	DriveBL,	tmotorVex393HighSpeed_HBridge, openLoop)

task main(){
	while(true){
		motor[DriveBR] = motor[DriveFR] = vexRT[Ch3] - vexRT[Ch1];
		motor[DriveBL] = motor[DriveFL] = vexRT[Ch3] + vexRT[Ch1];
	}
}
