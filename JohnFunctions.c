void haultYo (){
	motor[port1]=0;
	motor[port2]=0;
	motor[port3]=0;
	motor[port4]=0;
	motor[port5]=0;
	motor[port6]=0;
	motor[port7]=0;
	motor[port8]=0;
	motor[port9]=0;
	motor[port10]=0;
}


void goFoward (int pwr){
		motor[port4] = -pwr;
		motor[port3] = -pwr;
		motor[port2] = pwr;
		motor[port1] = pwr;
	}

	void goBackward (int pwr)
	{
		motor[port4] = pwr;
		motor[port3] = pwr;
		motor[port2] = -pwr;
		motor[port1] = -pwr;
	}
	void lTurn (int pwr)
	{
		motor[port4] = -pwr;
		motor[port3] = -pwr;
		motor[port2] = -pwr;
		motor[port1] = -pwr;
	}
	void rTurn (int pwr){
		motor[port4] = pwr;
		motor[port3] = pwr;
		motor[port2] = pwr;
		motor[port1] = pwr;
	}

	void flyEnable(int pwr){
	motor[port6] = -79;
	motor[port5] = -79;

	}

	void intakeEnable(int pwr){
	motor[port9] = -pwr;
	motor[port10] = -pwr;
}
void flyDisable(){
	motor[port6] = 0;
	motor[port5] = 0;

	}

	void intakeEnable(){
	motor[port9] = 0;
	motor[port10] = 0;
}

//PID STARTS HERE

void setfWheel(int pwr )
{
	motor[fly1]=motor[fly2]=pwr;
}


bool fWheelPidEnabled = false;
	int fWheelTarget=0;
task pid{
while (true){
	if (fWheelPidEnabled)
	setfWheel(myPID(fWheelTarget,fWheelPidValues,fWheel));
	wait1Msec(20);
}
}
