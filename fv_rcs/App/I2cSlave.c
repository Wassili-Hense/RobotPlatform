#include "I2cSlave.h"

#define I2C_SLAVE_RX_QUEUE_SIZE  8U

typedef struct {
  uint8_t size;
  uint8_t data[I2C_SLAVE_RX_SIZE];
} I2cSlave_RxFrame_t;

static I2C_HandleTypeDef *s_hi2c = 0;
static I2cSlave_RequestCallback_t s_requestCallback = 0;
static uint8_t s_rxData[I2C_SLAVE_RX_SIZE];
static uint8_t s_txData[I2C_SLAVE_TX_SIZE];
static uint8_t s_rxIndex = 0U;
static volatile uint8_t s_rxQueueHead = 0U;
static volatile uint8_t s_rxQueueTail = 0U;
static volatile uint8_t s_rxQueueCount = 0U;
static volatile uint8_t s_rxQueueOverflow = 0U;
static I2cSlave_RxFrame_t s_rxQueue[I2C_SLAVE_RX_QUEUE_SIZE];

static void I2cSlave_StartListen(void) {
  if (s_hi2c == 0) return;
  HAL_I2C_EnableListen_IT(s_hi2c);
}

static void I2cSlave_QueuePush(const uint8_t *data, uint8_t size) {
  uint8_t i;
  I2cSlave_RxFrame_t *frame;

  if ((data == 0) || (size == 0U)) return;

  if (s_rxQueueCount >= I2C_SLAVE_RX_QUEUE_SIZE) {
    s_rxQueueOverflow++;
    return;
  }

  frame = &s_rxQueue[s_rxQueueTail];
  frame->size = size;

  for (i = 0U; i < size; i++) {
    frame->data[i] = data[i];
  }

  s_rxQueueTail = (uint8_t)((s_rxQueueTail + 1U) % I2C_SLAVE_RX_QUEUE_SIZE);
  s_rxQueueCount++;
}

void I2cSlave_Init(I2C_HandleTypeDef *hi2c, I2cSlave_RequestCallback_t requestCallback) {
  s_hi2c = hi2c;
  s_requestCallback = requestCallback;
  s_rxIndex = 0U;
  s_rxQueueHead = 0U;
  s_rxQueueTail = 0U;
  s_rxQueueCount = 0U;
  s_rxQueueOverflow = 0U;
  I2cSlave_StartListen();
}

uint8_t I2cSlave_GetNextRxPacket(uint8_t *data, uint8_t *size) {
  uint8_t i;
  uint8_t frameSize;
  uint32_t primask;
  I2cSlave_RxFrame_t *frame;

  if ((data == 0) || (size == 0)) return 0U;

  primask = __get_PRIMASK();
  __disable_irq();

  if (s_rxQueueCount == 0U) {
    if (primask == 0U) __enable_irq();
    return 0U;
  }

  frame = &s_rxQueue[s_rxQueueHead];
  frameSize = frame->size;

  for (i = 0U; i < frameSize; i++) {
    data[i] = frame->data[i];
  }

  s_rxQueueHead = (uint8_t)((s_rxQueueHead + 1U) % I2C_SLAVE_RX_QUEUE_SIZE);
  s_rxQueueCount--;

  if (primask == 0U) __enable_irq();

  *size = frameSize;
  return 1U;
}

uint8_t I2cSlave_GetRxQueueOverflow(void) {
  return s_rxQueueOverflow;
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transferDirection, uint16_t addrMatchCode) {
  uint8_t txSize;
  (void) addrMatchCode;
  if (hi2c != s_hi2c) return;
  if (transferDirection == I2C_DIRECTION_TRANSMIT) {
    s_rxIndex = 0U;
    HAL_I2C_Slave_Seq_Receive_IT(s_hi2c, &s_rxData[s_rxIndex], 1U, I2C_NEXT_FRAME);
  } else {
    txSize = 0U;
    if (s_requestCallback != 0) {
      txSize = s_requestCallback(s_txData);
    }
    HAL_I2C_Slave_Seq_Transmit_IT(s_hi2c, s_txData, txSize, I2C_LAST_FRAME);
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c != s_hi2c) return;
  if (s_rxIndex > 0U) {
    I2cSlave_QueuePush(s_rxData, s_rxIndex);
  }
  s_rxIndex = 0U;
  I2cSlave_StartListen();
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c != s_hi2c) return;
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c != s_hi2c) return;
  s_rxIndex++;
  if (s_rxIndex < I2C_SLAVE_RX_SIZE) {
    HAL_I2C_Slave_Seq_Receive_IT(s_hi2c, &s_rxData[s_rxIndex], 1U, I2C_NEXT_FRAME);
  } else {
    I2cSlave_QueuePush(s_rxData, s_rxIndex);
    s_rxIndex = 0U;
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c != s_hi2c) return;

  if (s_rxIndex > 1U) {
    I2cSlave_QueuePush(s_rxData, s_rxIndex);
  }

  s_rxIndex = 0U;
  I2cSlave_StartListen();
}
