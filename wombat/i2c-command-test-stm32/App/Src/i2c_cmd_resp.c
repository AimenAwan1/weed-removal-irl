#include "main.h"
#include <stdbool.h>

#define SPEED_DATA_LEN (2 * sizeof(float))

float encoder_left_radps = 7.68;
float encoder_right_radps = 9.87;

float cmd_left_radps = 4.20;
float cmd_right_radps = 5.29;

uint8_t rxBuffer[SPEED_DATA_LEN];
uint8_t txBuffer[SPEED_DATA_LEN] = {1, 2, 3, 4, 5, 6, 7, 8};

uint32_t txCount = 0;
uint32_t totalTxCount = 0;

enum
{
    CMD_NONE = -1,
    CMD_READ_ENCODER,
    CMD_SET_SPEEDS
} active_cmd = CMD_NONE;

extern void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

bool yeehaw = false;

extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transferDirection, uint16_t addrMatchCode)
{
    if (transferDirection == I2C_DIRECTION_TRANSMIT)
    {
        // transmission from master
        yeehaw = true;
        HAL_I2C_Slave_Sequential_Receive_IT(hi2c, rxBuffer, 1, I2C_NEXT_FRAME);
    }
    else // I2C_DIRECTION_RECEIVE
    {
        if (active_cmd == CMD_READ_ENCODER)
        {
            // retrieves the most recent encoder data and transmits it
            // float *txSpeed = (float *)txBuffer;
            // txSpeed[0] = encoder_left_radps;
            // txSpeed[1] = encoder_right_radps;
            totalTxCount = SPEED_DATA_LEN;
            HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, txBuffer, 1, I2C_NEXT_FRAME);
        }
        else
        {
            // not following a read command
            Error_Handler();
        }
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (active_cmd == CMD_NONE)
    {
        active_cmd = rxBuffer[0];
        if (active_cmd == CMD_SET_SPEEDS)
        {
            // sets up another transaction to receive the commanded speeds
            HAL_I2C_Slave_Sequential_Receive_IT(hi2c, rxBuffer, 1, I2C_FIRST_FRAME);
        }
    }
    else if (active_cmd == CMD_SET_SPEEDS)
    {
        // handles receiving of the commanded speeds
        float *rxSpeed = (float *)rxBuffer;
        cmd_left_radps = rxSpeed[0];
        cmd_right_radps = rxSpeed[1];

        // finished handling the command
        active_cmd = CMD_NONE;
    }
    else
    {
        // protocol failure
        Error_Handler();
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // if (active_cmd == CMD_READ_ENCODER)
    // {
    //     ++txCount; // just finished transmitting a byte
    //     if (txCount >= SPEED_DATA_LEN)
    //     {
    //         active_cmd = CMD_NONE;
    //     }
    //     else
    //     {
    //         HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, rxBuffer + txCount, 1, I2C_NEXT_FRAME);
    //     }
    // }
    // else
    // {
    //     // protocol failure
    //     Error_Handler();
    // }
}

static bool flag_ovr = false;
static bool flag_af = false;
static bool flag_arlo = false;
static bool flag_berr = false;
static bool flag_txe = false;
static bool flag_rxne = false;
static bool flag_stopf = false;
static bool flag_add10 = false;
static bool flag_btf = false;
static bool flag_addr = false;
static bool flag_sb = false;
static bool flag_dualf = false;
static bool flag_gencall = false;
static bool flag_tra = false;
static bool flag_busy = false;
static bool flag_msl = false;

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    flag_ovr = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_OVR);
    flag_af = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF);
    flag_arlo = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ARLO);
    flag_berr = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BERR);
    flag_txe = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXE);
    flag_rxne = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE);
    flag_stopf = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF);
    flag_add10 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADD10);
    flag_btf = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF);
    flag_addr = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_ADDR);
    flag_sb = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_SB);
    flag_dualf = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_DUALF);
    flag_gencall = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_GENCALL);
    flag_tra = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TRA);
    flag_busy = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY);
    flag_msl = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_MSL);

    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_TRA);

    active_cmd = CMD_NONE;
    HAL_I2C_EnableListen_IT(hi2c);
}