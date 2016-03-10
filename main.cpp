/*
 * Virus Power Supply.cpp
 *
 * Created: 2/23/2016 1:36:09 PM
 * Author : ericsims
 */ 

#define F_CPU 8000000
#define BaudRate 9600
#define MYUBRR (F_CPU / 16 / BaudRate ) - 1


#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
//#include "i2c.h"

#define I2C_ADDR 0x02

bool have_battery;
int battery_voltage, battery_voltage_POST, USB_voltage;

void delayLong()
{
	unsigned int delayvar;
	delayvar = 0;
	while (delayvar <=  65500U)
	{
		asm("nop");
		delayvar++;
	}
}



unsigned char serialCheckRxComplete(void)
{
	return( UCSR0A & _BV(RXC0)) ;		// nonzero if serial data is available to read.
}

unsigned char serialCheckTxReady(void)
{
	return( UCSR0A & _BV(UDRE0) ) ;		// nonzero if transmit register is ready to receive new data.
}

unsigned char serialRead(void)
{
	while (serialCheckRxComplete() == 0)		// While data is NOT available to read
	{;;}
	return UDR0;
}

void serialWrite(unsigned char DataOut)
{
	while (serialCheckTxReady() == 0)		// while NOT ready to transmit
	{;;}
	UDR0 = DataOut;
}



void establishContact() {
	while (serialCheckRxComplete() == 0) {
		serialWrite('A');
		//	serialWrite(65U);
		delayLong();
		delayLong();
		delayLong();
		delayLong();
		delayLong();
		delayLong();
		delayLong();
	}
}

int main(void)
{
	have_battery = false;
	// initialization calls
	//i2cInit();
	//i2cSetLocalDeviceAddr(I2C_ADDR, TRUE);

 	/*Set baud rate */
 	UBRR0H = (unsigned char)(MYUBRR>>8);
 	UBRR0L = (unsigned char) MYUBRR;
 	/* Enable receiver and transmitter   */
 	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
 	/* Frame format: 8data, No parity, 1stop bit */
 	UCSR0C = (3<<UCSZ00);
 	
	
	// LED 4 as OUTPUT
	DDRB |= (1<<DDB0); 

	// configure ADC
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN);
	
    while (1) {
		ADMUX = MUX3;
		ADMUX |= (1 << REFS0);
		ADCSRA |= (1 << ADSC); // start ADC
		//while (ADCSRA & (1<<ADSC)); // wait for ADC...
		battery_voltage_POST = ADC; // read ADC
		have_battery = battery_voltage_POST > 500;

		if(have_battery) {
			PORTB &= ~(1<<PORTB0);
			_delay_ms(50);
			PORTB |= (1<<PORTB0);
			_delay_ms(950);
		} else {
			PORTB &= ~(1<<PORTB0);
			_delay_ms(100);
			PORTB |= (1<<PORTB0);
			_delay_ms(100);
		}
		char strgy[5] = { 0 };
		sprintf(strgy, "%d", battery_voltage_POST);
		serialWrite(strgy[0]);
		serialWrite(strgy[1]);
		serialWrite(strgy[2]);
		serialWrite(strgy[3]& 255U);
	}
}

