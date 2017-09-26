/*
 * uart.c
 *
 * Created: 27.01.2017 13:26:16
 *  Source: https://stratifylabs.co/embedded%20design%20tips/2013/10/02/Tips-A-FIFO-Buffer-Implementation/
 */ 

typedef struct {
	char * buf;
	int head;
	int tail;
	int size;
	int length;
} fifo_t;

#ifndef FIFO_H_
#define FIFO_H_

void fifo_init(fifo_t * f, char * buf, int size);
int fifo_read(fifo_t * f, void * buf, int nbytes);
int fifo_read_token(fifo_t * f, void * buf, int token);
int fifo_write(fifo_t * f, char item);

#endif /* FIFO_H_ */
