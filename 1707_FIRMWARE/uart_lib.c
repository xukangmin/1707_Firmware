/*
 * CFile1.c
 *
 * Created: 6/19/2018 8:25:41 AM
 *  Author: kangm
 */ 
 #include "uart_lib.h"
 #define F_CPU 3333333UL
 #define CLK_PER 3333333
 #define BAUD 9600
 #define MYBRR 64*F_CPU/16/BAUD
 #include <util/delay.h>




 void initUSART(){  // Initialize USART

	 PORTB_DIR |= (1 << PIN2_bp);

	 PORTB_OUT |= (1 << PIN2_bp);

	 PORTB_DIR &= ~(1 << PIN3_bp);

	 PORTB_DIR |= (1 << PIN0_bp);
	
	 USART0_BAUDH = (uint8_t)(MYBRR>>8); // Set the baud rate
	 USART0_BAUDL = (uint8_t)MYBRR;

	 USART0_CTRLA |= (1 << USART_RS4850_bp);  // enable RS485 Support
	 
	 USART0_CTRLA |= (1 << USART_RXCIE_bp);
	
	 USART0_CTRLB = (1 << USART_RXEN_bp) | (1 << USART_TXEN_bp);

	 USART0_CTRLC = (1 << USART_CHSIZE0_bp) | (1 << USART_CHSIZE1_bp);
	 
	 USART0_STATUS &= ( 1 << USART_RXCIE_bp);

	 
 }

 void transmit_byte( uint8_t data ){
	 while ( !( USART0_STATUS & (1 << USART_DREIF_bp)) );  // Wait for empty buffer.
	 USART0_TXDATAL = data;            // Put data into buffer.
 }

int receive_byte_non_block(uint8_t *rx) {
	if (bit_is_set(USART0_STATUS, USART_RXCIE_bp)) 
	{
		(*rx) = USART0_RXDATAL;
		return 1;
	} else {
		return 0;
	}
}

 uint8_t receive_byte(void) {
	 loop_until_bit_is_set(USART0_STATUS, USART_RXCIE_bp);       /* Wait for incoming data */
	 return USART0_RXDATAL;                                /* return register value */
 }

 void print_string(const char myString[]) {
	 uint8_t i = 0;
	 while (myString[i]) {
		 transmit_byte(myString[i]);
		 i++;
	 }
 }
 
 char nibbleToHexCharacter(uint8_t nibble) {
	 /* Converts 4 bits into hexadecimal */
	 if (nibble < 10) {
		 return ('0' + nibble);
	 }
	 else {
		 return ('A' + nibble - 10);
	 }
 }

 void GetHexString(uint8_t byte, char *out)
 {
	 uint8_t nibble;
	 nibble = (byte & 0b11110000) >> 4;
	 out[0] = nibbleToHexCharacter(nibble);
	 nibble = byte & 0b00001111;
	 out[1] = nibbleToHexCharacter(nibble);
 }

 void print_bytes(uint8_t myBytes[], uint8_t len, int check_sum_enable)
 {
	 uint8_t i = 0;
	 int checksum = 0;
	 char tmp[2];
	 
	 if (check_sum_enable == 1)
	 {
		 for(i = 0; i < len; i++)
		 {
			 transmit_byte(myBytes[i]);
			 checksum += myBytes[i];
		 }
		 checksum = checksum & 0xff;
		 GetHexString((uint8_t)checksum, tmp);
		 transmit_byte(tmp[0]);
		 transmit_byte(tmp[1]);
	 }
	 else{
		 for(i = 0; i < len; i++)
		 {
			 transmit_byte(myBytes[i]);
		 }
	 }

	 transmit_byte(0x0d);
	 
 }