// input buffer based on the tutorial by dekuNukem
// https://github.com/dekuNukem/STM32_tutorials/tree/master/lesson3_serial_recv_interrupt

#include "input_buf.h"
#include <string.h> // memset is declared here

void input_buf_reset(input_buf *ib) {
	if(NULL == ib) return; // check for null pointer. writing it this way to avoid an accidental assignment
	ib->curr_index = 0;
	memset(ib->buf, 0, BUF_SIZE);
}

void input_buf_add(input_buf *ib, uint8_t c) {
	if(NULL == ib) return;
	ib->buf[ib->curr_index] = c;
	if(ib->curr_index < BUF_SIZE-1) {
		ib->curr_index = ib->curr_index+1;
	} else {
		input_buf_reset(ib);
	}
}

uint8_t input_buf_ready(input_buf *ib) {
	if(NULL == ib) return 0;
	if(ib->buf[ib->curr_index-1] == '\n' || ib->buf[ib->curr_index-1] == '\r') {
		return 1;
	}
	return 0;
}

