#ifndef USB_H
#define USB_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_RX_BUF_SIZE 64u
#define USB_TX_BUF_SIZE 512u

void USBBegin(uint32_t baud);
void USBSetConnect(bool connected);
bool USBIsConnected(void);
bool USBReadLine(char *str, size_t cap);
void USBSendLine(const char *str);

#ifdef __cplusplus
}
#endif

#endif // USB_H
