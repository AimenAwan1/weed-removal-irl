#include "main.h"

#define SPEED_DATA_LEN (2 * sizeof(float))

float encoder_left_radps = 7.68;
float encoder_right_radps = 9.87;

float cmd_left_radps = 4.20;
float cmd_right_radps = 5.29;

uint8_t rxBuffer[SPEED_DATA_LEN];
uint8_t txBuffer[SPEED_DATA_LEN];

uint32_t rxCount = 0;

extern void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transferDirection, uint16_t addrMatchCode)
{
    // only reset during addressing
    rxCount = 0;

    if (transferDirection == I2C_DIRECTION_TRANSMIT) // if the master wants to transmit the data
    {
        // starts a read for the command byte
        HAL_I2C_Slave_Sequential_Receive_IT(hi2c, rxBuffer + (rxCount++), 1, I2C_FIRST_FRAME);
    }
    else
    {
        // master always needs to send the command byte first
        Error_Handler();
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (rxCount == 1)
    {
        enum
        {
            CMD_READ_ENCODERS,
            CMD_SET_SPEEDS,
        } cmd = rxBuffer[0];

        if (cmd == CMD_READ_ENCODERS)
        {
            // retrieves the most recent encoder data and transmits it
            float *txSpeed = (float *)txBuffer;
            txSpeed[0] = encoder_left_radps;
            txSpeed[1] = encoder_right_radps;
            HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, txBuffer, SPEED_DATA_LEN, I2C_LAST_FRAME_NO_STOP);
        }
        else
        {
            // sets up another transaction to receive the commanded speeds
            HAL_I2C_Slave_Sequential_Receive_IT(hi2c, rxBuffer, SPEED_DATA_LEN, I2C_FIRST_AND_LAST_FRAME);
            rxCount += SPEED_DATA_LEN;
        }
    }
    else if (rxCount == 1 + SPEED_DATA_LEN)
    {
        // handles receiving of the commanded speeds
        float *rxSpeed = (float *)rxBuffer;
        cmd_left_radps = rxSpeed[0];
        cmd_right_radps = rxSpeed[1];
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // do nothing on finishing
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}