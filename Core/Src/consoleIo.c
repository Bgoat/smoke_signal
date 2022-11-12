// Console IO is a wrapper between the actual in and output and the console code
// In an embedded system, this might interface to a UART driver.

#include "consoleIo.h"
#include <stdio.h>

eConsoleError ConsoleIoInit(void)
{
	return CONSOLE_SUCCESS;
}

// This is modified for the Wokwi RPi Pico simulator. It works fine
// but that's partially because the serial terminal sends all of the
// characters at a time without losing any of them. What if this function
// wasn't called fast enough?
eConsoleError ConsoleIoReceive(uint8_t *buffer, const uint32_t bufferLength, uint32_t *readLength)
{

	// ------- modified ----------
	uint16_t i;
	for(i=0; i<BUF_SIZE; i++) {
		if(uart_buf.buf[i] == 0) break;
		buffer[i] = (uint8_t)uart_buf.buf[i];
	}
	// -----------------

	*readLength = i;

	// original
	// uint32_t i = 0;
	// char ch;
	//	while (uart_is_readable(uart0))
	//	{
	//  	ch = uart_getc(uart0);
	//  	uart_putc(uart0, ch); // echo
	//		buffer[i] = (uint8_t) ch;
	//		i++;
	//	}
	//	*readLength = i;
	// ---

	return CONSOLE_SUCCESS;
}

eConsoleError ConsoleIoSendString(const char *buffer)
{
	//printf("%s", buffer); // original

	// --------- modified ---------

	// count the size (sizeof didn't work for me, so manually counting)
	uint16_t i;
	for(i=0; i<BUF_SIZE; i++) { // arbitrarily setting output limit to be the same as input limit
		if(buffer[i] == '\0') break;
	}

	// send it
	uint16_t the_size = i;
	HAL_UART_Transmit(&huart2, buffer, the_size, HAL_MAX_DELAY); // TODO: resolve this warning
	// ---------------------------

	return CONSOLE_SUCCESS;
}
