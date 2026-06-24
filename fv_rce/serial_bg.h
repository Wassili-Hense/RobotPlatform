#pragma once

#include <Arduino.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"

#ifndef SERIAL_BG_LINE_CAP
#define SERIAL_BG_LINE_CAP 32U
#endif

#ifndef SERIAL_BG_QUEUE_DEPTH
#define SERIAL_BG_QUEUE_DEPTH 8U
#endif

bool serial_bg_begin(uint32_t baud = 115200U,
                     bool connected = false,
                     BaseType_t coreId = 1,
                     UBaseType_t priority = 2,
                     uint32_t stackSize = 4096U);

void serial_bg_end(void);

bool serial_bg_is_connected(void);
void serial_bg_set_connected(bool connected);

bool serial_bg_send_line(const char* text);
bool serial_bg_receive_line(char* outLine, size_t outCap);
