#include "I2cSlave.h"

#define I2C_SLAVE_RX_SIZE  32U
#define I2C_SLAVE_TX_SIZE   2U

static I2C_HandleTypeDef *s_hi2c = 0;
static I2cSlave_RequestCallback_t s_requestCallback = 0;
static I2cSlave_ReceiveCallback_t s_receiveCallback = 0;
static uint8_t s_rxData[I2C_SLAVE_RX_SIZE];
static uint8_t s_txData[I2C_SLAVE_TX_SIZE];
static uint8_t s_rxIndex = 0U;

static void I2cSlave_StartListen(void)
{
    if (s_hi2c == 0)
    {
        return;
    }

    HAL_I2C_EnableListen_IT(s_hi2c);
}

void I2cSlave_Init(I2C_HandleTypeDef *hi2c,
                   I2cSlave_RequestCallback_t requestCallback,
                   I2cSlave_ReceiveCallback_t receiveCallback)
{
    s_hi2c = hi2c;
    s_requestCallback = requestCallback;
    s_receiveCallback = receiveCallback;
    s_rxIndex = 0U;
    I2cSlave_StartListen();
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transferDirection, uint16_t addrMatchCode)
{
    uint8_t txSize;

    (void)addrMatchCode;

    if (hi2c != s_hi2c)
    {
        return;
    }

    if (transferDirection == I2C_DIRECTION_TRANSMIT)
    {
        s_rxIndex = 0U;
        HAL_I2C_Slave_Seq_Receive_IT(s_hi2c, &s_rxData[s_rxIndex], 1U, I2C_NEXT_FRAME);
    }
    else
    {
        txSize = 0U;

        if (s_requestCallback != 0)
        {
            txSize = s_requestCallback(s_txData);
        }

        HAL_I2C_Slave_Seq_Transmit_IT(s_hi2c, s_txData, txSize, I2C_LAST_FRAME);
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != s_hi2c)
    {
        return;
    }

    if ((s_rxIndex > 0U) && (s_receiveCallback != 0))
    {
        s_receiveCallback(s_rxData, s_rxIndex);
    }

    s_rxIndex = 0U;
    I2cSlave_StartListen();
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != s_hi2c)
    {
        return;
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != s_hi2c)
    {
        return;
    }

    s_rxIndex++;

    if (s_rxIndex < I2C_SLAVE_RX_SIZE)
    {
        HAL_I2C_Slave_Seq_Receive_IT(s_hi2c, &s_rxData[s_rxIndex], 1U, I2C_NEXT_FRAME);
    }
    else
    {
        if (s_receiveCallback != 0)
        {
            s_receiveCallback(s_rxData, s_rxIndex);
        }
        s_rxIndex = 0U;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != s_hi2c)
    {
        return;
    }

    s_rxIndex = 0U;
    I2cSlave_StartListen();
}
