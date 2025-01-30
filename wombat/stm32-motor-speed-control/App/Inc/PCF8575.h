/*
 * PCF8575.h
 *
 *  Created on: Jan 22, 2025
 *      Author: Jason
 */

#ifndef INC_PCF8575_H_
#define INC_PCF8575_H_

#include "main.h"

#define FLOAT_CONV_MULT_FACTOR 1000.0

typedef struct{
	I2C_HandleTypeDef *hi2c; /* I2C peripheral instance */
	uint8_t isRead; /* 1 = Read, 0 = Write  */
	uint8_t addr;   /* 7 bit address must be shifted left by 1 */
}PCF8575_inst;

void writeGPIOExp(PCF8575_inst *PCF8575, int16_t intWrite); // writes int16_t to two I2C uint8_t messages
int16_t readGPIOExp(PCF8575_inst *PCF8575); // reads int16_t to two I2C uint8_t messages

int16_t floatToInt16(float fl);   // Float is first multiplied by FLOAT_CONV_MULT_FACTOR, then converted to int
float int16ToFloat(int16_t i_16); // Int is divided by FLOAT_CONV_MULT_FACTOR, converted to a float

#endif /* INC_PCF8575_H_ */
