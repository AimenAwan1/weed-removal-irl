#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "main.h"

typedef struct{
	TIM_HandleTypeDef    *htim_motor; /* timer instance for the pwm signal*/
	uint32_t           htim_motor_f_ch; /* backwards PWM channel */
	uint32_t           htim_motor_b_ch; /* forward PWM channel */
	GPIO_TypeDef      *en_pin_port;  /* forward enable pin*/
	uint16_t         en_pin_number;  /* forward enable pin*/
}motor_inst;

void enable_motor(motor_inst *motor);
void disable_motor(motor_inst *motor);
// void motor_init(motor_inst *motor);
void set_speed_open(motor_inst *motor, float duty_cycle_percent);
void set_speed_zero(motor_inst *motor);
#endif /* INC_MOTOR_CONTROL_H_ */
