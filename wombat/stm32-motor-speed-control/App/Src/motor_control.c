/*
 * Created on: Jan 16, 2025
 *
 * motor_control.c
 *
 * Author Jason Gao
 *
 * Library to control BTS7960 half-bridge motor driver breakout with PWM.
 * Adapted from code by: Yerkebuan Massalim, Steppeschool
 *
 */

#include "motor_control.h"

/*
 * Enables the motor driver
 */
void enable_motor(motor_inst *motor)
{
	HAL_GPIO_WritePin(motor -> en_pin_port, motor -> en_pin_number, GPIO_PIN_SET);
}

/*
 * Disables the motor driver (both forwards and backwards disabled)
 */
void disable_motor(motor_inst *motor)
{
	HAL_GPIO_WritePin(motor -> en_pin_port, motor -> en_pin_number, GPIO_PIN_RESET);
}

/*
 * Sets motor PWM to prescribed PWM
 */
void set_speed_open(motor_inst *motor, float duty_cycle_percent)
{
	// Check and set limits for PWM percent
	if(duty_cycle_percent > 100.0)
	{
		duty_cycle_percent = 100.0;
	}
	else if(duty_cycle_percent < -100.0)
	{
		duty_cycle_percent = -100.0;
	}

	// Set PWM
	if(duty_cycle_percent > 0)
	{
		__HAL_TIM_SET_COMPARE(motor -> htim_motor, motor -> htim_motor_b_ch, 0);
		__HAL_TIM_SET_COMPARE(motor -> htim_motor, motor -> htim_motor_f_ch,
				duty_cycle_percent/100 * (motor ->htim_motor->Instance->ARR+1) );

	}
	else
	{
		__HAL_TIM_SET_COMPARE(motor -> htim_motor, motor -> htim_motor_f_ch, 0);
		__HAL_TIM_SET_COMPARE(motor -> htim_motor, motor -> htim_motor_b_ch,
						(-duty_cycle_percent)/100 * (motor ->htim_motor->Instance->ARR+1) );
	}
}

/*
 * Turns both forward and backward PWM off
 */
void set_speed_zero(motor_inst *motor)
{
	__HAL_TIM_SET_COMPARE(motor -> htim_motor, motor -> htim_motor_f_ch, 0);
	__HAL_TIM_SET_COMPARE(motor -> htim_motor, motor -> htim_motor_b_ch, 0);
}
