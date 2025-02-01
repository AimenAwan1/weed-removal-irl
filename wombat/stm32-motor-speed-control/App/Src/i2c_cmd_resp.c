#include "main.h"
#include "motor_encoder.h"
#include <stdbool.h>

#define SPEED_DATA_LEN (2 * sizeof(float))

extern encoder_inst motor_L_enc;
extern encoder_inst motor_R_enc;

static float cmd_left_radps = 4.20;
static float cmd_right_radps = 5.29;

static uint8_t rxBuffer[SPEED_DATA_LEN];
static uint8_t txBuffer[SPEED_DATA_LEN];

static uint32_t txCount = 0;
static uint32_t totalTxCount = 0;

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

bool failed = false;

extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transferDirection, uint16_t addrMatchCode)
{
    if (transferDirection == I2C_DIRECTION_TRANSMIT)
    {
        // transmission from master
        yeehaw = true;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxBuffer, 1, I2C_FIRST_FRAME);
    }
    else // I2C_DIRECTION_RECEIVE
    {
        if (active_cmd == CMD_READ_ENCODER)
        {
            // retrieves the most recent encoder data and transmits it
            float *txSpeed = (float *)txBuffer;
            txSpeed[0] = motor_L_enc.velocity;
            txSpeed[1] = motor_R_enc.velocity;
            HAL_I2C_Slave_Seq_Transmit_IT(hi2c, txBuffer, 8, I2C_LAST_FRAME);
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
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, rxBuffer, SPEED_DATA_LEN, I2C_FIRST_FRAME);
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
    if (active_cmd == CMD_READ_ENCODER)
    {
        active_cmd = CMD_NONE;
    }
    else
    {
        // protocol failure
        Error_Handler();
    }
}

// static bool error_none = false;
// static bool error_berr = false;
// static bool error_arlo = false;
// static bool error_af = false;
// static bool error_ovr = false;
// static bool error_dma = false;
// static bool error_timeout = false;
// static bool error_size = false;
// static bool error_dma_param = false;
// static bool error_wrong_start = false;

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    // switch (HAL_I2C_GetError(hi2c))
    // {
    // case HAL_I2C_ERROR_BERR:
    //     error_berr = true;
    //     break;
    // case HAL_I2C_ERROR_ARLO:
    //     error_arlo = true;
    //     break;
    // case HAL_I2C_ERROR_AF:
    //     error_af = true;
    //     break;
    // case HAL_I2C_ERROR_OVR:
    //     error_ovr = true;
    //     break;
    // case HAL_I2C_ERROR_DMA:
    //     error_dma = true;
    //     break;
    // case HAL_I2C_ERROR_TIMEOUT:
    //     error_timeout = true;
    //     break;
    // case HAL_I2C_ERROR_SIZE:
    //     error_size = true;
    //     break;
    // case HAL_I2C_ERROR_DMA_PARAM:
    //     error_dma_param = true;
    //     break;
    // case HAL_I2C_WRONG_START:
    //     error_wrong_start = true;
    //     break;
    // case HAL_I2C_ERROR_NONE:
    // default:
    //     error_none = true;
    //     break;
    // }

    active_cmd = CMD_NONE;
    HAL_I2C_EnableListen_IT(hi2c);
}