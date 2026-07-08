#include "USB.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define USB_TX_TASK_STACK 3072u
#define USB_TX_TASK_PRIO 1u
static uint32_t s_baud = 115200u;

struct RingBuf {
  char *buf;
  size_t size;
  volatile size_t head;
  volatile size_t tail;
};

static char s_rxStorage[USB_RX_BUF_SIZE];
static char s_txStorage[USB_TX_BUF_SIZE];

static RingBuf s_rx = { s_rxStorage, USB_RX_BUF_SIZE, 0, 0 };
static RingBuf s_tx = { s_txStorage, USB_TX_BUF_SIZE, 0, 0 };

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_connected = false;
static volatile bool s_uartStarted = false;
static TaskHandle_t s_txTask = nullptr;

static inline size_t rbNext(const RingBuf *rb, size_t pos) {
  ++pos;
  return (pos >= rb->size) ? 0 : pos;
}

static inline bool rbIsEmptyLocked(const RingBuf *rb) {
  return rb->head == rb->tail;
}

static inline bool rbIsFullLocked(const RingBuf *rb) {
  return rbNext(rb, rb->head) == rb->tail;
}

static size_t rbUsedLocked(const RingBuf *rb) {
  if (rb->head >= rb->tail) {
    return rb->head - rb->tail;
  }

  return rb->size - rb->tail + rb->head;
}

static size_t rbFreeLocked(const RingBuf *rb) {
  // One byte is intentionally kept unused to distinguish full and empty states.
  return rb->size - 1u - rbUsedLocked(rb);
}

static void rbClearLocked(RingBuf *rb) {
  rb->head = 0;
  rb->tail = 0;
}

static bool rbPushLocked(RingBuf *rb, char ch) {
  const size_t next = rbNext(rb, rb->head);
  if (next == rb->tail) return false;

  rb->buf[rb->head] = ch;
  rb->head = next;
  return true;
}

static bool rbPopLocked(RingBuf *rb, char *ch) {
  if (rbIsEmptyLocked(rb)) {
    return false;
  }

  *ch = rb->buf[rb->tail];
  rb->tail = rbNext(rb, rb->tail);
  return true;
}

static bool rbHasLineLocked(const RingBuf *rb) {
  size_t pos = rb->tail;
  while (pos != rb->head) {
    if (rb->buf[pos] == '\n') return true;
    pos = rbNext(rb, pos);
  }
  return false;
}

static size_t strLen8(const char *str) {
  size_t len = 0;
  while (str[len] != 0) {
    ++len;
  }
  return len;
}

static void USBOnReceiveCb(void) {
  while (Serial.available() > 0) {
    const int v = Serial.read();
    if (v < 0) break;

    portENTER_CRITICAL_ISR(&s_lock);
    if (s_connected) {
      (void)rbPushLocked(&s_rx, (char)v);
    }
    portEXIT_CRITICAL_ISR(&s_lock);
  }

}

static bool txPopChunk(char *chunk, size_t cap, size_t *len) {
  *len = 0;

  portENTER_CRITICAL(&s_lock);
  while (*len < cap && !rbIsEmptyLocked(&s_tx)) {
    (void)rbPopLocked(&s_tx, &chunk[*len]);
    ++(*len);
  }
  portEXIT_CRITICAL(&s_lock);

  return *len > 0;
}

static void USBTxTask(void *arg) {
  (void)arg;
  char chunk[64];
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));

    if (!s_connected || !s_uartStarted) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    while (s_connected && s_uartStarted) {
      const int writable = Serial.availableForWrite();
      if (writable <= 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        break;
      }

      const size_t cap = ((size_t)writable < sizeof(chunk)) ? (size_t)writable : sizeof(chunk);
      size_t len = 0;
      if (!txPopChunk(chunk, cap, &len)) {
        break;
      }

      (void)Serial.write(chunk, len);
      // Yield so tasks with the same or lower priority are not starved.
      taskYIELD();
    }
  }
}

static void USBStartUart(void) {
  if (s_uartStarted) return;

  // timeout_ms = 0 keeps begin() from waiting for UART initialization timeouts.
  Serial.begin(s_baud, SERIAL_8N1, -1, -1, false, 0UL, 1U);
  Serial.setRxTimeout(4);
  Serial.onReceive(USBOnReceiveCb);
  s_uartStarted = true;
}

static void USBStopUart(void) {
  if (!s_uartStarted) return;

  Serial.end();  // For low-power disconnect, dropping the remaining TX bytes is intentional.
  s_uartStarted = false;
}

void USBBegin(uint32_t baud) {
  s_baud = baud;
  xTaskCreatePinnedToCore(
      USBTxTask,
      "USB_Tx",
      USB_TX_TASK_STACK,
      nullptr,
      USB_TX_TASK_PRIO,
      &s_txTask,
      ARDUINO_RUNNING_CORE);
  s_connected = true;
  USBStartUart();
}

void USBSetConnect(bool connected) {
  if (connected) {
    portENTER_CRITICAL(&s_lock);
    s_connected = true;
    portEXIT_CRITICAL(&s_lock);
    USBStartUart();
    Serial.setDebugOutput(true);
  } else {
    Serial.setDebugOutput(false);
    portENTER_CRITICAL(&s_lock);
    s_connected = false;
    rbClearLocked(&s_rx);
    rbClearLocked(&s_tx);
    portEXIT_CRITICAL(&s_lock);
    USBStopUart();
  }
  if (s_txTask) {
    xTaskNotifyGive(s_txTask);
  }
}

bool USBIsConnected(void) {
  return s_connected;
}

bool USBReadLine(char *str, size_t cap) {
  if (!str || cap == 0 || !s_connected) {
    return false;
  }

  portENTER_CRITICAL(&s_lock);
  const bool hasLine = rbHasLineLocked(&s_rx);
  if (!hasLine) {
    portEXIT_CRITICAL(&s_lock);
    return false;
  }

  size_t outLen = 0;
  char ch = 0;

  while (rbPopLocked(&s_rx, &ch)) {
    if (ch == '\n') break;
    if (ch == '\r') continue;
    if (outLen + 1 < cap) {
      str[outLen++] = ch;
    }
  }

  str[outLen] = 0;
  portEXIT_CRITICAL(&s_lock);
  return true;
}

void USBSendLine(const char *str) {
  if (!str || !s_connected) {
    return;
  }

  bool notify = false;
  const size_t len = strLen8(str);
  const size_t need = len + 2u;  // Add "\r\n" automatically.

  portENTER_CRITICAL(&s_lock);

  if (rbFreeLocked(&s_tx) >= need) {
    for (size_t i = 0; i < len; ++i) {
      (void)rbPushLocked(&s_tx, str[i]);
    }
    (void)rbPushLocked(&s_tx, '\r');
    (void)rbPushLocked(&s_tx, '\n');
    notify = true;
  } else if (!rbIsFullLocked(&s_tx)) {
    (void)rbPushLocked(&s_tx, '!');
    notify = true;
  }

  portEXIT_CRITICAL(&s_lock);

  if (notify && s_txTask) {
    xTaskNotifyGive(s_txTask);
  }
}
