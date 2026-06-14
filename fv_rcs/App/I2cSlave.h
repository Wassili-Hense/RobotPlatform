#ifndef I2CSLAVE_H
#define I2CSLAVE_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t (*I2cSlave_RequestCallback_t)(uint8_t *outData);
typedef void (*I2cSlave_ReceiveCallback_t)(const uint8_t *data, uint16_t size);

void I2cSlave_Init(I2C_HandleTypeDef *hi2c,
                   I2cSlave_RequestCallback_t requestCallback,
                   I2cSlave_ReceiveCallback_t receiveCallback);

#ifdef __cplusplus
}
#endif

#endif /* I2CSLAVE_H */
