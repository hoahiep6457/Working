#include "stm32f4xx.h"
#include "main.h"
#include <stdlib.h>
#include <math.h>
#include "pid.h"
float error;
float diff;
float PID_Adjustment(pid_t *pid, float setpoint, float target)
{
	error = setpoint - target;
	diff =  error - pid->lastError;
	pid->lastError =  error; //save value for next time
	//calculate PID contents
	pid->Kp_Value = Kp*error;
	pid->Ki_Value += Ki*error*pid_sampletime;
	pid->Kd_Value = Kd*diff/pid_sampletime;
	//Update ouput
	pid->Output = pid->Kp_Value + pid->Ki_Value + pid->Kd_Value;
	if (pid->Output > Outmax){
		pid->Output = Outmax;
	}
	if (pid->Output < Outmin){
		pid->Output = Outmin;
	}
	return pid->Output;
}
void PID_Init(void)
{

}