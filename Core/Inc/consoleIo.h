// Console IO is a wrapper between the actual in and output and the console code

#ifndef CONSOLE_IO_H
#define CONSOLE_IO_H

#include <stdint.h>

// ------- added ------
#include "stm32f4xx_hal.h"
#include "input_buf.h"
extern UART_HandleTypeDef huart2;
extern input_buf uart_buf;
// -----------------

typedef enum {CONSOLE_SUCCESS = 0u, CONSOLE_ERROR = 1u } eConsoleError;

eConsoleError ConsoleIoInit(void);

eConsoleError ConsoleIoReceive(uint8_t *buffer, const uint32_t bufferLength, uint32_t *readLength);
eConsoleError ConsoleIoSendString(const char *buffer); // must be null terminated

#endif // CONSOLE_IO_H
