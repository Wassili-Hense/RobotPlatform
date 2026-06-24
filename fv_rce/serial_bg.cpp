#include "serial_bg.h"

#include <string.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

typedef struct {
    char text[SERIAL_BG_LINE_CAP];
} serial_bg_msg_t;

static QueueHandle_t s_rxQueue = NULL;
static QueueHandle_t s_txQueue = NULL;
static SemaphoreHandle_t s_serialMutex = NULL;
static TaskHandle_t s_rxTaskHandle = NULL;
static TaskHandle_t s_txTaskHandle = NULL;

static volatile bool s_started = false;
static volatile bool s_connected = false;

static uint32_t s_baud = 115200U;
static BaseType_t s_coreId = 1;
static UBaseType_t s_priority = 2;
static uint32_t s_stackSize = 4096U;

static char s_rxLine[SERIAL_BG_LINE_CAP];
static size_t s_rxLineLen = 0U;

static void serial_bg_reset_rx_assembler(void)
{
    s_rxLineLen = 0U;
    s_rxLine[0] = '\0';
}

static void serial_bg_flush_queue(QueueHandle_t queue)
{
    serial_bg_msg_t dummy;

    if (queue == NULL) {
        return;
    }

    while (xQueueReceive(queue, &dummy, 0U) == pdTRUE) {
    }
}

static void serial_bg_push_rx_line(void)
{
    serial_bg_msg_t msg;

    if ((s_rxQueue == NULL) || (s_rxLineLen == 0U)) {
        serial_bg_reset_rx_assembler();
        return;
    }

    memcpy(msg.text, s_rxLine, s_rxLineLen);
    msg.text[s_rxLineLen] = '\0';
    (void)xQueueSend(s_rxQueue, &msg, 0U);
    serial_bg_reset_rx_assembler();
}

static void serial_bg_service_rx(void)
{
    while (Serial.available() > 0) {
        const int value = Serial.read();
        char ch;

        if (value < 0) {
            break;
        }

        ch = (char)value;
        if ((ch == '\r') || (ch == '\n')) {
            serial_bg_push_rx_line();
            continue;
        }

        if ((s_rxLineLen + 1U) < SERIAL_BG_LINE_CAP) {
            s_rxLine[s_rxLineLen++] = ch;
        } else {
            serial_bg_reset_rx_assembler();
        }
    }
}

static void serial_bg_on_receive(void)
{
    if (s_rxTaskHandle != NULL) {
        xTaskNotifyGive(s_rxTaskHandle);
    }
}

static void serial_bg_rx_task(void* arg)
{
    (void)arg;

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!s_connected) {
            continue;
        }

        if ((s_serialMutex != NULL) && (xSemaphoreTake(s_serialMutex, pdMS_TO_TICKS(20U)) == pdTRUE)) {
            serial_bg_service_rx();
            xSemaphoreGive(s_serialMutex);
        }
    }
}

static void serial_bg_tx_task(void* arg)
{
    serial_bg_msg_t msg;

    (void)arg;

    for (;;) {
        if (xQueueReceive(s_txQueue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!s_connected) {
            continue;
        }

        if ((s_serialMutex != NULL) && (xSemaphoreTake(s_serialMutex, pdMS_TO_TICKS(20U)) == pdTRUE)) {
            if (s_connected) {
                Serial.println(msg.text);
            }
            xSemaphoreGive(s_serialMutex);
        }
    }
}

static bool serial_bg_start_serial_locked(void)
{
    Serial.begin(s_baud);
    (void)Serial.setRxFIFOFull(1U);
    (void)Serial.setRxTimeout(2U);
    Serial.onReceive(serial_bg_on_receive, true);
    serial_bg_reset_rx_assembler();
    s_connected = true;
    if (s_rxTaskHandle != NULL) {
        xTaskNotifyGive(s_rxTaskHandle);
    }
    return true;
}

static void serial_bg_stop_serial_locked(void)
{
    Serial.flush();
    Serial.end();
    serial_bg_flush_queue(s_rxQueue);
    serial_bg_flush_queue(s_txQueue);
    serial_bg_reset_rx_assembler();
    s_connected = false;
}

bool serial_bg_begin(uint32_t baud,
                     bool connected,
                     BaseType_t coreId,
                     UBaseType_t priority,
                     uint32_t stackSize)
{
    if (s_started) {
        return true;
    }

    s_rxQueue = xQueueCreate(SERIAL_BG_QUEUE_DEPTH, sizeof(serial_bg_msg_t));
    if (s_rxQueue == NULL) {
        return false;
    }

    s_txQueue = xQueueCreate(SERIAL_BG_QUEUE_DEPTH, sizeof(serial_bg_msg_t));
    if (s_txQueue == NULL) {
        vQueueDelete(s_rxQueue);
        s_rxQueue = NULL;
        return false;
    }

    s_serialMutex = xSemaphoreCreateMutex();
    if (s_serialMutex == NULL) {
        vQueueDelete(s_txQueue);
        vQueueDelete(s_rxQueue);
        s_txQueue = NULL;
        s_rxQueue = NULL;
        return false;
    }

    s_baud = baud;
    s_coreId = coreId;
    s_priority = priority;
    s_stackSize = stackSize;
    s_connected = false;
    serial_bg_reset_rx_assembler();

    if (xTaskCreatePinnedToCore(serial_bg_rx_task,
                                "SerialBgRx",
                                s_stackSize,
                                NULL,
                                s_priority,
                                &s_rxTaskHandle,
                                s_coreId) != pdPASS) {
        vSemaphoreDelete(s_serialMutex);
        vQueueDelete(s_txQueue);
        vQueueDelete(s_rxQueue);
        s_serialMutex = NULL;
        s_txQueue = NULL;
        s_rxQueue = NULL;
        return false;
    }

    if (xTaskCreatePinnedToCore(serial_bg_tx_task,
                                "SerialBgTx",
                                s_stackSize,
                                NULL,
                                s_priority,
                                &s_txTaskHandle,
                                s_coreId) != pdPASS) {
        vTaskDelete(s_rxTaskHandle);
        s_rxTaskHandle = NULL;
        vSemaphoreDelete(s_serialMutex);
        vQueueDelete(s_txQueue);
        vQueueDelete(s_rxQueue);
        s_serialMutex = NULL;
        s_txQueue = NULL;
        s_rxQueue = NULL;
        return false;
    }

    s_started = true;

    if (connected) {
        serial_bg_set_connected(true);
    }

    return true;
}

void serial_bg_end(void)
{
    if (!s_started) {
        return;
    }

    if ((s_serialMutex != NULL) && (xSemaphoreTake(s_serialMutex, pdMS_TO_TICKS(20U)) == pdTRUE)) {
        if (s_connected) {
            serial_bg_stop_serial_locked();
        }
        xSemaphoreGive(s_serialMutex);
    }

    if (s_rxTaskHandle != NULL) {
        xTaskNotifyGive(s_rxTaskHandle);
        vTaskDelete(s_rxTaskHandle);
        s_rxTaskHandle = NULL;
    }

    if (s_txTaskHandle != NULL) {
        serial_bg_msg_t msg;
        memset(&msg, 0, sizeof(msg));
        if (s_txQueue != NULL) {
            (void)xQueueSend(s_txQueue, &msg, 0U);
        }
        vTaskDelete(s_txTaskHandle);
        s_txTaskHandle = NULL;
    }

    if (s_serialMutex != NULL) {
        vSemaphoreDelete(s_serialMutex);
        s_serialMutex = NULL;
    }

    if (s_txQueue != NULL) {
        vQueueDelete(s_txQueue);
        s_txQueue = NULL;
    }

    if (s_rxQueue != NULL) {
        vQueueDelete(s_rxQueue);
        s_rxQueue = NULL;
    }

    s_connected = false;
    s_started = false;
    serial_bg_reset_rx_assembler();
}

bool serial_bg_is_connected(void)
{
    return s_connected;
}

void serial_bg_set_connected(bool connected)
{
    if (!s_started || (s_serialMutex == NULL)) {
        s_connected = connected;
        return;
    }

    if (connected == s_connected) {
        return;
    }

    if (xSemaphoreTake(s_serialMutex, pdMS_TO_TICKS(20U)) != pdTRUE) {
        return;
    }

    if (connected) {
        (void)serial_bg_start_serial_locked();
    } else {
        serial_bg_stop_serial_locked();
    }

    xSemaphoreGive(s_serialMutex);
}

bool serial_bg_send_line(const char* text)
{
    serial_bg_msg_t msg;
    size_t len;

    if ((!s_started) || (!s_connected) || (s_txQueue == NULL) || (text == NULL)) {
        return false;
    }

    len = strnlen(text, SERIAL_BG_LINE_CAP);
    if ((len == 0U) || (len >= SERIAL_BG_LINE_CAP)) {
        return false;
    }

    memcpy(msg.text, text, len);
    msg.text[len] = '\0';
    return (xQueueSend(s_txQueue, &msg, 0U) == pdTRUE);
}

bool serial_bg_receive_line(char* outLine, size_t outCap)
{
    serial_bg_msg_t msg;

    if ((outLine == NULL) || (outCap == 0U) || (s_rxQueue == NULL)) {
        return false;
    }

    if (xQueueReceive(s_rxQueue, &msg, 0U) != pdTRUE) {
        return false;
    }

    strncpy(outLine, msg.text, outCap - 1U);
    outLine[outCap - 1U] = '\0';
    return true;
}
