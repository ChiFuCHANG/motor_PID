#include "pid.h"

void pid_control(PID *pid, float pos, float *tau)
{
	pid->P_term = (float)TARGET_POS - pos;
	pid->I_term += pid->P_term;
	if (pid->I_term >= pid->max_Integral)
	{
		pid->I_term = pid->max_Integral;
	}
	else if (pid->I_term <= pid->max_Integral)
	{
		pid->I_term = -pid->max_Integral;
	}
	pid->D_term = pid->P_term - pid->err_pre;
	pid->err_pre = pid->P_term;
	*tau = pid->Kp * pid->P_term + pid->Ki * pid->I_term + pid->Kd * pid->D_term;
	if (*tau >= pid->max_PID)
	{
		*tau = pid->max_PID;
	}
	else if (*tau <= pid->min_PID)
	{
		*tau = pid->min_PID;
	}
}
