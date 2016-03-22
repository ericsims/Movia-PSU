/*
 * Virus Power Supply.cpp
 *
 * Created: 2/23/2016 1:36:09 PM
 * Author : ericsims
 */ 

// TODO: Turn off the virus lights and stuff when it low battery





#define F_CPU 8000000
#define BaudRate 9600
#define MYUBRR (F_CPU / 16 / BaudRate ) - 1

#define prog_char char

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <i2c.c>

#define I2C_ADDR 0x09<<1

bool have_battery, have_usb;
bool PWR_SW;
int battery_voltage, battery_voltage_POST, USB_voltage;

uint8_t localBuffer[] = "Pascal is cool!!Pascal is Cool!!";
uint8_t localBufferLength = 0x8;

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

uint8_t i2cSlaveTransmitService(uint8_t transmitDataLengthMax, uint8_t* transmitData)
{
	char strgy[8] = { 0 };
	sprintf(strgy, "%d", battery_voltage);
	
	for(int i=0; i<8; i++)
	{
		*transmitData++ = strgy[i];
	}

	return 8;
}
int main(void)
{
	have_battery = false;
	have_usb = false;
	PWR_SW = false;
	// initialization calls
	i2cInit();
	i2cSetLocalDeviceAddr(I2C_ADDR, true);

	i2cSetSlaveTransmitHandler( i2cSlaveTransmitService );

 	/*Set baud rate */
 	UBRR0H = (unsigned char)((MYUBRR)>>8);
 	UBRR0L = (unsigned char) MYUBRR;
 	/* Enable receiver and transmitter   */
 	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
 	/* Frame format: 8data, No parity, 1stop bit */
 	UCSR0C = (3<<UCSZ00);
 	
	
	// LED 4 as OUTPUT
	DDRB |= (1<<DDB0); 
	// Charge pin as OUTPUT
	DDRB |= (1<<DDB1);
	PORTB |= (1<<DDB1);
	// SW1 as INPUT and enable pullup res
	DDRB &= ~(1<<DDB2);
	PORTB |= (1<<DDB2);

	// configure ADC
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN);
	
    while (1) {
		ADMUX = MUX2;
		ADMUX |= (1 << REFS0);
		ADCSRA |= (1 << ADSC); // start ADC
		//while (ADCSRA & (1<<ADSC)); // wait for ADC...
		battery_voltage = ADC; // read ADC
		
		have_battery = battery_voltage > 500;
		have_usb = false;
		PWR_SW = ~(PINB >> DDB2)&0x01;		 
		
		if(have_battery && PWR_SW) {
			PORTB &= ~(1<<PORTB0);
			_delay_ms(500);
		} else if(have_battery && ~PWR_SW) {
			PORTB &= ~(1<<PORTB0);
			_delay_ms(100);
			PORTB |= (1<<PORTB0);
			_delay_ms(400);
		} else {
			PORTB |= (1<<PORTB0);
			_delay_ms(500);
		}
		char strgy[5] = { 0 };
		sprintf(strgy, "%d", battery_voltage);
		serialWrite(strgy[0]);
		serialWrite(strgy[1]);
		serialWrite(strgy[2]);
		serialWrite(strgy[3]);
		serialWrite('\n');
	}
}
