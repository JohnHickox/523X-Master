#pragma config(Sensor, in1,    speedy,         sensorPotentiometer)
#pragma config(Sensor, dgtl1,  ,               sensorTouch)
#pragma config(Sensor, dgtl4,  drive,          sensorQuadEncoder)
#pragma config(Sensor, dgtl6,  drive1,         sensorQuadEncoder)
#pragma config(Sensor, dgtl8,  ,               sensorTouch)
#pragma config(Sensor, dgtl11, fWheel,         sensorQuadEncoder)
#pragma config(Motor,  port1,           bRDrive,       tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           FRdrive,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           bLDrive,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           fLDrive,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           fly1,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           fly2,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           lift1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           lift2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           intake1,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          intake2,       tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"
#include "Genral.c"
#include "chassis.c"
#include "JohnFunctions.c"
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
	SensorValue(fWheel)=0;
	setPIDValues();
	fWheelPidEnabled=true;
	drivePidEnabled=true;


  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
startTask(drivePid);

	flyEnable(79);
	delay(2000);
	intakeEnable(127);
	delay(1000);
	lTurn(127);
	delay(800);
	motor[port1] = 127;
	motor[port2] = 127;
	motor[port3] = -120;
	motor[port4] = -120;
	delay(2250);
	haultYo();
	//2 FLAGS TURNED
	// START OF CAP BALL INTAKE
	goBackward(127);
	wait1Msec(1720);
	haultYo();
	rTurn(127);
	wait1Msec(400);
	haultYo();
	goBackward(127);
	wait1Msec(1000);
	intakeEnable(127);
	wait1Msec(250);
	haultYo();
	intakeEnable(127);
	wait1Msec(200);
	goFoward(127);
	wait1Msec(1250);
	haultYo();

//while(SensorValue[touchy] == 0)
{






}

  AutonomousCodePlaceholderForTesting();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
startTask(pid);
setPIDValues();

	int drive;

//main driving code
 while (true)
  {
  		//drive
  	if(vexRT[Btn5U])
  	{
		drive = 1;
	}
	else if(vexRT[Btn5D])
	{
	drive = -1;
	}

	if(drive == 1)
	{
		motor[port4] = vexRT[Ch3];
		motor[port3] = vexRT[Ch3];
		motor[port2] = -vexRT[Ch2];
		motor[port1] = -vexRT[Ch2];
}
else if(drive == -1)
{
		motor[port4] = -vexRT[Ch2];
		motor[port3] = -vexRT[Ch2];
		motor[port2] = vexRT[Ch3];
		motor[port1] = vexRT[Ch3];
}
		//intake
		if(vexRT[Btn5UXmtr2]) // if(vexRT[Btn5UXmrt2])
		{
		motor[port9] = -127;
		motor[port10] = -127;
		}
		else if(vexRT[Btn5DXmtr2])
		{
		motor[port9] = 127;
		motor[port10] = 127;
		}
		else
		{
		motor[port9] = 0;
		motor[port10] = 0;
		}

		//flywheel

		if(vexRT[Btn6UXmtr2])
		{
		motor[port6] = abs(vexRT[Ch2Xmtr2]);
		motor[port5] = abs(vexRT[Ch2Xmtr2]);
	}
	else
	{
		motor[port6] = -abs(vexRT[Ch3Xmtr2]);
		motor[port5] = -abs(vexRT[Ch3Xmtr2]);
	}
		//lift
		if(vexRT[Btn6U])
		{
		motor[port7] = 127;
		motor[port8] = 127;
		}
		else if(vexRT[Btn6D])
		{
		motor[port7] = -127;
		motor[port8] = -127;
		}
		else
			{
			motor[port7] = 0;
			motor[port8] = 0;
		}

		//boop
		if(vexRT[Btn8D])
		{
		motor[port7] = 127;
		motor[port8] = 127;
		wait1Msec(500);
		motor[port7] = -127;
		motor[port8] = -127;
		wait1Msec(500);
		}

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

		}
}
