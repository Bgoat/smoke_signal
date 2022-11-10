// input buffer based on the tutorial by dekuNukem
// https://github.com/dekuNukem/STM32_tutorials/tree/master/lesson3_serial_recv_interrupt

#ifndef INC_INPUT_BUF_H_
#define INC_INPUT_BUF_H_

#include <stdio.h>

// can send max-2 chars (2 for \r\n)
// unclear why in coolterm it's max-3...
#define BUF_SIZE 128 // this space is allocated in the RAM

typedef struct {
	int16_t curr_index;
	uint8_t buf[BUF_SIZE];
} input_buf;

void input_buf_reset(input_buf *ib);
void input_buf_add(input_buf *ib, uint8_t c);
uint8_t input_buf_ready(input_buf *ib);

#endif
