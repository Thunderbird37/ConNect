#pragma once

#include <stddef.h>
#include <stdint.h>

void usbHostInit();
bool usbHostInitialized();
void usbHostTask();
bool usbHostConnected();
void usbHostGetDriverName(char* buffer, size_t bufferSize);
int usbHostReadByte();
size_t usbHostWrite(const uint8_t* data, size_t len);
void usbHostWriteByte(uint8_t b);
void usbHostSetLineCoding(uint32_t baud_rate);
uint8_t usbHostDataBits();
uint8_t usbHostParity();
void usbHostSetSignals(bool dtr, bool rts);
uint32_t usbHostDroppedBytes();
size_t usbHostRxQueueCapacity();
