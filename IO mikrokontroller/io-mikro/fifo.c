/*
 * uart.c
 *
 * Created: 27.01.2017 13:26:16
 * Source: https://stratifylabs.co/embedded%20design%20tips/2013/10/02/Tips-A-FIFO-Buffer-Implementation/
 */ 

#include "fifo.h"
#include <avr/cpufunc.h>

//This initializes the FIFO structure with the given buffer and size
void fifo_init(fifo_t * f, char * buf, int size){
	f->head = 0;
	f->tail = 0;
	f->size = size;
	f->buf = buf;
	f->length = 0;
}

//This reads nbytes bytes from the FIFO
//The number of bytes read is returned
int fifo_read(fifo_t * f, void * buf, int nbytes){
	int i;
	char * p;
	p = buf;
	for(i=0; i < nbytes; i++){
		if( f->tail != f->head ){ //see if any data is available
			*p++ = f->buf[f->tail];  //grab a byte from the buffer
			f->tail++;  //increment the tail
			f->length--;
			if( f->tail == f->size ){  //check for wrap-around
				f->tail = 0;
			}
		} else {
			return i; //number of bytes read
		}
	}
	return nbytes;
}

//This writes up to nbytes bytes to the FIFO
//If the head runs in to the tail, not all bytes are written
//The number of bytes written is returned
int fifo_write(fifo_t * f, char item){
	//first check to see if there is space in the buffer
	if( (f->head + 1 == f->tail) || ( (f->head + 1 == f->size) && (f->tail == 0) ) ){
		return 0; //no more room
	} else {
		f->buf[f->head] = item;
		f->head++;  //increment the head
		f->length++;
		if( f->head == f->size ){  //check for wrap-around
			f->head = 0;
		}
	}

	return 1;
}

//This reads bytes from the fifo until token is found, if the token is not found nothing is returned
//The number of bytes read is returned
int fifo_read_token(fifo_t * f, void * buf, int token){
	int i=0;
	int old_tail = f->tail;
	int old_length = f->length;
	char * p;
	p = buf;
	while(1) {
		if( f->tail != f->head ){ //see if any data is available
			*p++ = f->buf[f->tail];  //grab a byte from the buffer
			f->tail++;  //increment the tail
			f->length--;
			i++;
			if( f->tail == f->size ){  //check for wrap-around
				f->tail = 0;
			}
			if(*(p-1) == token) {
				_NOP();
				return i;
			}
		} else {
			f->tail= old_tail;
			f->length = old_length;
			return 0; 
		}
	}
}