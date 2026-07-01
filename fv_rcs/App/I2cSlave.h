#ifndef I2CSLAVE_H
#define I2CSLAVE_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_SLAVE_RX_SIZE  32U
#define I2C_SLAVE_TX_SIZE  32U

typedef uint8_t (*I2cSlave_RequestCallback_t)(uint8_t *outData);

void I2cSlave_Init(I2C_HandleTypeDef *hi2c, I2cSlave_RequestCallback_t requestCallback);
uint8_t I2cSlave_GetNextRxPacket(uint8_t *data, uint8_t *size);
uint8_t I2cSlave_GetRxQueueOverflow(void);

#ifdef __cplusplus
}
#endif

#endif /* I2CSLAVE_H */
