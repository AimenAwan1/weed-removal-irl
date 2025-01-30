/*
 * PCF8575.c
 *
 *  Created on: Jan 22, 2025
 *      Author: Jason
 */

#include "PCF8575.h"

const static float float_conv_factor = 1000.0;

void writeGPIOExp(PCF8575_inst *PCF8575, int16_t intWrite)
{
	uint8_t buf[2];
	buf[0] = intWrite;
	intWrite = intWrite >> 8;
	buf[1] = intWrite;
	if(PCF8575 -> isRead == 0) // Check that the GPIO expander is supposed to be an input before reading any data
	{
		HAL_I2C_Master_Transmit_IT(PCF8575 -> hi2c, PCF8575 -> addr, buf, 2);
	}
}

int16_t readGPIOExp(PCF8575_inst *PCF8575)
{
	int16_t intRead;
	uint8_t buf[2];
	if(PCF8575 -> isRead == 1) // Check that the GPIO expander is supposed to be an output before writing any data
	{
		HAL_I2C_Master_Receive_IT(PCF8575 -> hi2c, PCF8575 -> addr, buf, 1);
	}
	intRead = (buf[1] << 8) | buf[0];
	return intRead;
}

// Float is first multiplied by FLOAT_CONV_MULT_FACTOR, then converted to int
int16_t floatToInt16(float fl)
{
	fl = fl * float_conv_factor;
	return (int)fl;
}

// Int is divided by FLOAT_CONV_MULT_FACTOR, converted to a float
float int16ToFloat(int16_t i_16)
{
	return (i_16 / float_conv_factor);
}
