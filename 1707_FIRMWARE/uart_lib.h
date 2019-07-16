/*
 * IncFile1.h
 *
 * Created: 6/19/2018 8:25:48 AM
 *  Author: kangm
 */ 


#ifndef UART_LIB_H
#define UART_LIB_H

#include "avr/io.h"

void initUSART(void);

void transmit_byte( uint8_t data );

uint8_t receive_byte(void);

int receive_byte_non_block(uint8_t *rx);

 void print_string(const char myString[]);
 
 void print_bytes(uint8_t myBytes[], uint8_t len, int check_sum_enable);

#endif /* INCFILE1_H_ */