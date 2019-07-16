/*
 * uart_test_prog.c
 *
 * Created: 7/15/2019 10:24:58 AM
 * Author : kangmin
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#define F_CPU 3333333UL
#define CLK_PER 3333333

#define TWI_BAUDRATE 100000

#define TWI_SBAUD (CLK_PER/(2*TWI_BAUDRATE) - 5)

#include <util/delay.h>
#include <avr/sfr_defs.h>
#include "twi_master.h"
#include "uart_lib.h"

#define PB5 5

#define PB_SCL_PIN 0
#define PB_SDA_PIN 1

#define DEFAULT_ADDR "02A007"
#define DEFAULT_CAL_A 0.0f
#define DEFAULT_CAL_B 1.0f
#define DEFAULT_CAL_C 0.0f
#define DEFAULT_CAL_D 0.0f
#define DEFAULT_CAL_E 1.0f
#define DEFAULT_CAL_F 0.0f
#define DEFAULT_SETTING "000000"

#define STS3X_DEFAULT_ADDR			   0x4A
#define STS3X_MEAS_HIGHREP_STRETCH   0x2C06
#define STS3X_MEAS_MEDREP_STRETCH    0x2C0D
#define STS3X_MEAS_LOWREP_STRETCH    0x2C10
#define STS3X_MEAS_HIGHREP           0x2400
#define STS3X_MEAS_MEDREP            0x240B
#define STS3X_MEAS_LOWREP            0x2416
#define STS3X_READSTATUS             0xF32D
#define STS3X_CLEARSTATUS            0x3041
#define STS3X_SOFTRESET              0x30A2
#define STS3X_HEATEREN               0x306D
#define STS3X_HEATERDIS              0x3066

#define STS3X_FETCH_DATA			 0xE000
#define STS3X_STOP_PERIODIC_DAQ      0x3093

#define STS3X_PERIODIC_DAQ_0_5_HIGH  0x2032
#define STS3X_PERIODIC_DAQ_0_5_MED   0x2024
#define STS3X_PERIODIC_DAQ_0_5_LOW	 0x202F
#define STS3X_PERIODIC_DAQ_1_HIGH    0x2130
#define STS3X_PERIODIC_DAQ_1_MED     0x2126
#define STS3X_PERIODIC_DAQ_1_LOW     0x212D
#define STS3X_PERIODIC_DAQ_2_HIGH    0x2236
#define STS3X_PERIODIC_DAQ_2_MED     0x2220
#define STS3X_PERIODIC_DAQ_2_LOW     0x222B
#define STS3X_PERIODIC_DAQ_4_HIGH    0x2334
#define STS3X_PERIODIC_DAQ_4_MED     0x2322
#define STS3X_PERIODIC_DAQ_4_LOW     0x2329
#define STS3X_PERIODIC_DAQ_10_HIGH   0x2737
#define STS3X_PERIODIC_DAQ_10_MED    0x2721
#define STS3X_PERIODIC_DAQ_10_LOW    0x272A


#define ADDRESS_LEN 6
#define CMD_LEN2 2
#define CMD_LEN3 2

#define ALWAYS_RESP_ADDR "999999"

#define MAX_BUFFER_SIZE 32
#define DATA_RETAIN_SIZE 30
#define DATA_SIZE_PER_AVG_LEVEL 3  // max avg level = 9, so max data size = 27
#define TIMER0_TOP_VALUE 0xFF

#define ERROR_DATA 999.99f

#define EEPROM_INIT_STATUS_BYTE 	0x00
#define EEPROM_ADDR_START_BYTE  	0x01
#define EEPROM_CAL_TEMP_START_BYTE  0x07
#define EEPROM_CAL_HUM_START_BYTE	0x13
#define EEPROM_SETTING_START_BYTE   0x1F

#define INITED 0x01

TWI_Master_t twi_master;
TWI_t inst;

char addr[6],setting[6];
uint8_t write_enabled = 0;
int check_sum_enable = 0;
int delay = 0;
int avg_level = 0;


int data_size = 0;
volatile uint8_t result = 0;

float temp_data;
uint8_t rev = 0x00;
volatile uint8_t v = 0, m = 0, n = 0;
char recv[MAX_BUFFER_SIZE];
int send_enable = 0;
int send_size = 0;
uint8_t tmp[sizeof(float)];
int fstr_size = 0;
float cal_a,cal_b,cal_c,cal_d,cal_e,cal_f;

volatile int data_index = 0;
volatile uint8_t temp_flag = 0;
volatile unsigned long timer0_count = 0;
volatile int current_data_index = 0;
volatile int handle_uart_flag = 0;
volatile int data_overflow_flag = 0;
volatile float data_arr[DATA_RETAIN_SIZE] = {0.0};

volatile uint8_t read_temp_flag = 1;
volatile uint8_t data_buf[16];

TWI_Master_t twi_master;
TWI_t inst;

enum Cmd {
	RD = 0,
	RT,
	RH,
	RC,
	RE,
	WE,
	RS,
	SC,
	RA,
	SA,
	SS
};
const int cmd_len[] = {
	2,
	2,
	2,
	3,
	3,
	2,
	2,
	3,
	2,
	9,
	9
};

const char *cmd_list[] = {
	"RD",
	"RT",
	"RH",
	"RC",
	"RE",
	"WE",
	"RS",
	"SC",
	"RA",
	"SA",
	"SS"
};


ISR(USART0_RXC_vect)
{
	if (handle_uart_flag == 0) 
	{
		rev = USART0_RXDATAL;
		if (rev == '#' && v == 0)
		{
			v = 1;
			memset(recv,0,sizeof(recv));
			recv[v - 1] = rev;
		} 
		else if (v > 0) {
			recv[v] = rev;
			v++;

			if (v >= MAX_BUFFER_SIZE)
			{
				v = 0;
			}
			
			if (rev == 0x0d)
			{
				handle_uart_flag = 1;
			}
		} else {
			v = 0;
		}
	} else {
		USART0_RXDATAL;  // discard data when processing
	}
	USART0_STATUS |= ( 1 << USART_RXCIE_bp);
}

ISR(TWI0_TWIM_vect)
{
	/* Needed for the TWI to complete transactions */
	TWI_MasterInterruptHandler(&twi_master);
}


ISR(RTC_CNT_vect) {
	read_temp_flag = 1;
	RTC_INTFLAGS |= (1 << RTC_OVF_bp);
}

uint8_t crc8(const uint8_t *data, int len) {
  /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL = 0x31;
  uint8_t crc = 0xFF;

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

unsigned char EEPROM_read(unsigned int ucAddress)
{
	while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);

	return *(uint8_t *)(EEPROM_START + ucAddress);
}

void EEPROM_write(unsigned int ucAddress, unsigned char ucData)
{

	while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);

	*(uint8_t *)(EEPROM_START + ucAddress) = ucData;

	CCP = CCP_SPM_gc;
	NVMCTRL.CTRLA = NVMCTRL_CMD_PAGEERASEWRITE_gc;
}



void init_config()
{
	int i;
	
	
	cal_a = DEFAULT_CAL_A;
	cal_b = DEFAULT_CAL_B;
	cal_c = DEFAULT_CAL_C;
	
	cal_d = DEFAULT_CAL_D;
	cal_e = DEFAULT_CAL_E;
	cal_f = DEFAULT_CAL_F;

	for(i = 0; i < 6; i++)
	{
		addr[i] = DEFAULT_ADDR[i];
		EEPROM_write(EEPROM_ADDR_START_BYTE + i, DEFAULT_ADDR[i]);
	}
	
	for(i = 0; i < 6; i++)
	{
		setting[i] = DEFAULT_SETTING[i];
		EEPROM_write(EEPROM_SETTING_START_BYTE + i, DEFAULT_SETTING[i]);
	}
	check_sum_enable = DEFAULT_SETTING[0] - 0x30;
	delay = DEFAULT_SETTING[1] - 0x30;
	avg_level = DEFAULT_SETTING[2] - 0x30;
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_a,sizeof(float));
		EEPROM_write(EEPROM_CAL_TEMP_START_BYTE + i, tmp[i]);
	}
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_b,sizeof(float));
		EEPROM_write(EEPROM_CAL_TEMP_START_BYTE + sizeof(float) + i, tmp[i]);
	}
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_c,sizeof(float));
		EEPROM_write(EEPROM_CAL_TEMP_START_BYTE + 2 * sizeof(float) + i, tmp[i]);
	}

	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_d,sizeof(float));
		EEPROM_write(EEPROM_CAL_HUM_START_BYTE + i, tmp[i]);
	}
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_e,sizeof(float));
		EEPROM_write(EEPROM_CAL_HUM_START_BYTE + sizeof(float) + i, tmp[i]);
	}
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_f,sizeof(float));
		EEPROM_write(EEPROM_CAL_HUM_START_BYTE + 2 * sizeof(float) + i, tmp[i]);
	}
}

uint8_t read_init()
{
	return EEPROM_read(EEPROM_INIT_STATUS_BYTE);
}

void load_config()
{
	int i;
	for(i = 0; i < 6; i++)
	{
		addr[i] = EEPROM_read(EEPROM_ADDR_START_BYTE + i);
		setting[i] = EEPROM_read(EEPROM_SETTING_START_BYTE + i);
	}
	check_sum_enable = setting[0] - 0x30;
	delay = setting[1] - 0x30;
	avg_level = setting[2] - 0x30;
	
	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_TEMP_START_BYTE + m);
	}
	memcpy(&cal_a, tmp, sizeof(float));
	
	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_TEMP_START_BYTE + 0x04 + m);
	}
	memcpy(&cal_b, tmp, sizeof(float));

	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_TEMP_START_BYTE + 0x08 + m);
	}
	memcpy(&cal_c, tmp, sizeof(float));


	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_HUM_START_BYTE + m);
	}
	memcpy(&cal_d, tmp, sizeof(float));
	
	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_HUM_START_BYTE + 0x04 + m);
	}
	memcpy(&cal_e, tmp, sizeof(float));

	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_HUM_START_BYTE + 0x08 + m);
	}
	memcpy(&cal_f, tmp, sizeof(float));
}

void init_i2c() {

	PORTMUX_CTRLB |= PORTMUX_TWI0_ALTERNATE_gc;

	volatile uint8_t baud_rate = (CLK_PER/(2*TWI_BAUDRATE) - 5);
	
	TWI_MasterInit(&twi_master, &TWI0, (TWI_RIEN_bm | TWI_WIEN_bm), baud_rate);
}

void enable_periodic_daq() {
	
	data_buf[0] = STS3X_PERIODIC_DAQ_1_HIGH >> 8;
	data_buf[1] = (uint8_t)STS3X_PERIODIC_DAQ_1_HIGH;
	
	TWI_MasterWrite(&twi_master, STS3X_DEFAULT_ADDR, data_buf, 2);
	
	_delay_ms(50);
}

void get_periodic_data(float *temp_out) {
	float temp = 0;
	
	data_buf[0] = STS3X_FETCH_DATA >> 8;
	data_buf[1] = (uint8_t)STS3X_FETCH_DATA;
	
	TWI_MasterWrite(&twi_master, STS3X_DEFAULT_ADDR, data_buf, 2);
	
	_delay_ms(50);
	
	TWI_MasterRead(&twi_master, STS3X_DEFAULT_ADDR, 3);

	_delay_ms(10);

	if (crc8(twi_master.readData, 2) == twi_master.readData[2])
	{
		uint16_t tmp_raw = (twi_master.readData[0] * 256) + twi_master.readData[1];
		
		temp = (float)tmp_raw;
		
		temp = -45.0 + (175.0 * temp / 65535.0);
		} else {
		temp = 999.99;
	}



	(*temp_out) = temp;
}

void get_avg_data(float *temp) {
	
	float sum = 0;
	int data_count = 0;
	int i = 0;
	int start_index = 0;
	if (data_index == 0) {
		(*temp) = ERROR_DATA;
		return;
	}
	if (avg_level == 0) {
		(*temp) = data_arr[data_index - 1];
	} else {
		if (data_overflow_flag && data_index - avg_level * DATA_SIZE_PER_AVG_LEVEL < 0) { // data overflowed
			for(i = 0; i < data_index; i++) {
				sum	+= data_arr[i];
				data_count++;
			}
			start_index = DATA_RETAIN_SIZE - avg_level * DATA_SIZE_PER_AVG_LEVEL + data_index;
			for(i = start_index; i < DATA_RETAIN_SIZE; i++) {
				sum += data_arr[i];
				data_count++;
			}
		
			
		} else {
			start_index = data_index - avg_level * DATA_SIZE_PER_AVG_LEVEL;
			if (start_index < 0) start_index = 0;
			for(i = start_index; i < data_index; i++) {
				sum += data_arr[i];
				data_count++;
			}
		}
		
		
		(*temp) = sum / (float)data_count;
	}

	
	
}

void handle_uart_buffer(){
	if (handle_uart_flag == 1) {
		if (memcmp(recv + 1, ALWAYS_RESP_ADDR,ADDRESS_LEN) == 0 // verify address
		|| memcmp(recv + 1, addr, ADDRESS_LEN) == 0) {
			recv[0] = '*';
			if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[RD], strlen(cmd_list[RD])) == 0
			&& v == ADDRESS_LEN + 2 + cmd_len[RD])  // RD cmd #02A001RD\r
			{
				write_enabled = 0;
				recv[v - 1] = '=';
				//read_temp(&temp_data); // discard first data
				//get_periodic_data(&temp_data);
				get_avg_data(&temp_data);
				temp_data =  cal_a + temp_data * cal_b + cal_c * temp_data * temp_data;
				fstr_size = snprintf(recv + v, MAX_BUFFER_SIZE - v, "%.3f", temp_data);
				v += fstr_size;
				send_size = v;
				send_enable = 1;
			}
			else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[WE], strlen(cmd_list[WE])) == 0
			&& v == ADDRESS_LEN + 2 + cmd_len[WE])
			{
				write_enabled = 1;
				send_enable = 1;
				send_size = v - 1;
			}
			else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[RC], strlen(cmd_list[RC])) == 0
			&& v == ADDRESS_LEN + 2 + cmd_len[RC])
			{
				if (recv[v - 2] >= 0x41 && recv[v - 2] <= 0x43)
				{
					write_enabled = 0;
					recv[v - 1] = '=';
					data_size = 8;
					
					if (recv[v - 2] == 'A')
					{
						temp_data = cal_a;
					}
					else if (recv[v - 2] == 'B'){
						temp_data = cal_b;
					}
					else if (recv[v - 2] == 'C'){
						temp_data = cal_c;
					}

					if (fabs(temp_data) > 1e6) {
						fstr_size = snprintf(recv + v, MAX_BUFFER_SIZE - v, "%e", temp_data);
					}
					else {
						fstr_size = snprintf(recv + v, MAX_BUFFER_SIZE - v, "%f", temp_data);
					}
					

					send_size = fstr_size + v;
					send_enable = 1;
				}
			}
			else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[RS], strlen(cmd_list[RS])) == 0
			&& v == ADDRESS_LEN + 2 + cmd_len[RS])
			{
				write_enabled = 0;
				data_size = 6;
				recv[v - 1] = '=';
				
				for(m = 0; m < data_size; m++)
				{
					setting[m] = EEPROM_read(EEPROM_SETTING_START_BYTE + m);
					recv[v + m] = setting[m];
				}
				
				send_enable = 1;
				send_size = v +	data_size;
				
			}
			else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[SC], strlen(cmd_list[SC])) == 0
			&& write_enabled == 1)
			{
				if (recv[9] >= 0x41 && recv[9] <= 0x43)
				{
					write_enabled = 0;
					
					temp_data = atof(recv + 11);

					uint8_t add_bytes = 0x00;
					if (recv[9] == 'A')
					{
						add_bytes = 0x00;
						cal_a = temp_data;
					}
					else if (recv[9] == 'B'){
						add_bytes = 0x04;
						cal_b = temp_data;
					}
					else if (recv[9] == 'C'){
						add_bytes = 0x08;
						cal_c = temp_data;
					}
					
					
					memcpy(tmp, &temp_data, sizeof(float));
					for(m = 0; m < 4; m++)
					{
						EEPROM_write(EEPROM_CAL_TEMP_START_BYTE + add_bytes + m, tmp[m]);
					}
					send_enable = 1;
					send_size = v - 1;
				}
			}
			else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[RA], strlen(cmd_list[RA])) == 0
			&& v == ADDRESS_LEN + 2 + cmd_len[RA])
			{
				write_enabled = 0;
				data_size = 6;
				recv[v - 1] = '=';
				
				for(m = 0; m < data_size; m++)
				{
					//addr[m] = EEPROM_read(EEPROM_ADDR_START_BYTE + m);
					recv[v + m] = addr[m];
				}
				
				send_enable = 1;
				send_size = v +	data_size;
				
			}
			else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[SA], strlen(cmd_list[SA])) == 0
			&& v == ADDRESS_LEN + 2 + cmd_len[SA]
			&& write_enabled == 1)
			{
				write_enabled = 0;
				uint8_t n1 = 0;
				uint8_t n2 = 1;
				for(n1 = 0; n1 < 6; n1++)
				{
					if (recv[10 + n1] < 0x30 ||
					(recv[10 + n1] > 0x39 && recv[10 + n1] < 0x41) ||
					recv[10 + n1] > 0x5A)
					{
						n2 = 0;
						break;
					}
				}
				if (n2 == 1)
				{
					
					for(n1 = 0; n1 < 6; n1++)
					{
						addr[n1] = recv[10 + n1];
						EEPROM_write(EEPROM_ADDR_START_BYTE + n1, recv[10 + n1]);
					}
					send_enable = 1;
					send_size = v - 1;
				}
			}
			else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[SS], strlen(cmd_list[SS])) == 0
			&& v == ADDRESS_LEN + 2 + cmd_len[SS]
			&& write_enabled == 1)
			{
				write_enabled = 0;
				uint8_t n1 = 0;
				uint8_t n2 = 1;
				for(n1 = 0; n1 < 6; n1++)
				{
					if (recv[10 + n1] > 0x39 || recv[10 + n1] < 0x30)
					{
						n2 = 0;
						break;
					}
				}
				if (n2 == 1)
				{
					
					for(n1 = 0; n1 < 6; n1++)
					{
						setting[n1] = recv[10 + n1];
						EEPROM_write(EEPROM_SETTING_START_BYTE + n1, recv[10 + n1]);
					}
					send_enable = 1;
					send_size = v - 1;
					check_sum_enable = setting[0] - 0x30;
					delay = setting[1] - 0x30;
					avg_level = setting[2] - 0x30;
				}
			}

			if (send_enable == 1)
			{
				send_enable = 0;
				if (delay != 0)
				{
					for(n = 0; n < delay; n++)
					{
						_delay_us(1040);
					}
				}
				print_bytes((uint8_t *)recv, send_size, check_sum_enable);
			}
		}
		else {
			v = 0;

			return;
		}

		v = 0;
		handle_uart_flag = 0;
	}
}

void read_temp(float *temp_out)
{
	float temp = 0;
	
	data_buf[0] = STS3X_MEAS_HIGHREP >> 8;
	data_buf[1] = (uint8_t)STS3X_MEAS_HIGHREP;
	
	TWI_MasterWrite(&twi_master, STS3X_DEFAULT_ADDR, data_buf, 2);
	
	_delay_ms(50);
	
	TWI_MasterRead(&twi_master, STS3X_DEFAULT_ADDR, 3);

	_delay_ms(10);


	uint16_t tmp_raw = (twi_master.readData[0] * 256) + twi_master.readData[1];
	
	temp = (float)tmp_raw;
	
	temp = -45.0 + (175.0 * temp / 65535.0);


	(*temp_out) = temp;
}

int main(void)
{	
	RTC.CLKSEL = 0x00;
	
	loop_until_bit_is_clear(RTC.STATUS, RTC_PERBUSY_bp);
		
	RTC_PER = 0x8000;  // set overflow to 32768, since clock is 32768Hz

	RTC.INTCTRL = (1 << RTC_OVF_bp);
  
	RTC.CTRLA = 0x01;
  
	if (read_init() != INITED)
	{
		EEPROM_write(EEPROM_INIT_STATUS_BYTE, INITED);
		init_config();
	}
	
	load_config();
    	
	initUSART();
	init_i2c();
	
	enable_periodic_daq();
	
	
	sei();
	
	while (1) 
    {
		handle_uart_buffer();
		
		if (read_temp_flag) {
			read_temp_flag = 0;
			// read temp and save to arr
			if (data_index >= DATA_RETAIN_SIZE) {
				data_index = 0;
				data_overflow_flag = 1;
			}
			

			if (current_data_index < data_index) {
				current_data_index = data_index;
			}

			get_periodic_data(&data_arr[data_index++]);
		}
    }
}

