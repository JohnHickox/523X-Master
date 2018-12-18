typedef struct
{

	float kP;
	float kI;
	float kD;

	float error;
	float lastError;
	float integral;

} PIDValues;

PIDValues drivePIDValues;
PIDValues fWheelPidValues;
PIDValues turnPidValues;
void setPIDValues()
{

	drivePIDValues.kP = 0.0;
	drivePIDValues.kI = 0.00;
	drivePIDValues.kD = 0.0;
	fWheelPidValues.kP = 0.0;
	fWheelPidValues.kI = 0.00;
	fWheelPidValues.kD = 0.0;
	turnPIDValues.kP = 0.0;
	turnPIDValues.kI = 0.00;
	turnPIDValues.kD = 0.00;
}

int myPID(int setpoint, PIDValues &PIDStruct, tSensors errorSource){

	int power;
	float derivative;

	PIDStruct.error = setpoint - SensorValue[ errorSource ];
	derivative = PIDStruct.error - PIDStruct.lastError;
	power = PIDStruct.error*PIDStruct.kP + derivative*PIDStruct.kD;

	PIDStruct.lastError = PIDStruct.error;

	return power;
}

int myPID(int setpoint, PIDValues &PIDStruct, int errorSource){

	int power;
	float derivative;

	PIDStruct.error = setpoint -  errorSource;
	derivative = PIDStruct.error - PIDStruct.lastError;
	power = PIDStruct.error*PIDStruct.kP + derivative*PIDStruct.kD;

	PIDStruct.lastError = PIDStruct.error;

	return power;
}
