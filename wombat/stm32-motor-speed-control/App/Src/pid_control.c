#include "pid_control.h"

void set_pid(pid_instance *pid, float p, float i, float d)
{
    pid->error_integral = 0;
    pid->k_p = 0;
    pid->k_i = 0;
    pid->k_d = 0;
}

void reset_pid(pid_instance *pid)
{
    pid->error_integral = 0;
    pid->last_error = 0;
}

pid_typedef apply_pid(pid_instance *pid, float input_error)
{
	pid ->error_integral += input_error;
	if(pid->error_integral > pid ->integral_max)
	{
		pid->error_integral = pid ->integral_max;
	}
	if(pid->error_integral < -pid ->integral_max)
	{
		pid->error_integral = -pid ->integral_max;
	}
	if ( pid ->sam_rate == 0)
	{
		return pid_numerical;
	}
	pid ->pwm_output = pid ->k_p * input_error +
			pid ->k_i * (pid->error_integral) / pid ->sam_rate +
			pid ->k_d * pid ->sam_rate * (input_error - pid->last_error);

	if(pid->pwm_output >= pid->pid_max)
	{
		pid->pwm_output = pid->pid_max;
	}
	if(pid->pwm_output <= -pid->pid_max)
	{
		pid->pwm_output = -pid->pid_max;
	}
	pid->last_error = input_error;

	return pid_ok;

}