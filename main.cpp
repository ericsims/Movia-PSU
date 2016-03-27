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
#include <math.h>
#include <stdio.h>
#include <i2c.c>

bool have_battery, have_usb;
bool PWR_SW;
bool charging;
uint16_t battery_voltage, battery_voltage_POST, USB_voltage;

uint8_t registerPosition = 0x00;

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

void i2cSlaveReceiveService(uint8_t receiveDataLength, uint8_t* receiveData)
{
	if(*receiveData == 4) {
		have_usb = 1;
	} else if(*receiveData == 5) {
		have_usb = 0;
	} else {
		registerPosition = *receiveData;
	}
}

uint8_t i2cSlaveTransmitService(uint8_t transmitDataLengthMax, uint8_t* transmitData)
{
	char strgy[8] = { 0 };
	uint8_t data = 0;
	switch (registerPosition) {
	case 0: // reg
		data = registerPosition;
		break;
	case 1: // bat
		data = battery_voltage;
		break;
	case 2: // usb
		data = USB_voltage;
		break;
	case 3: // powersw
		data = PWR_SW;
		break;
	}
	
	for(uint8_t i = 0; i < 8; i++)
	{
		*transmitData++ = data;
	}

	return 8;
}

void startCharge() {
	charging = true;
	PORTD |= (1<<DDB5);
	_delay_ms(100);
	PORTD |= (1<<DDB7);
	_delay_ms(100);
	PORTD |= (1<<DDB6);
	_delay_ms(100);
	PORTB |= (1<<DDB1);
}
void stopCharge() {
	PORTB &= ~(1<<DDB1);
	_delay_ms(100);
	PORTD &= ~(1<<DDB6);
	_delay_ms(100);
	PORTD &= ~(1<<DDB7);
	_delay_ms(100);
	PORTD &= ~(1<<DDB5);
	_delay_ms(100);
	charging = false;
}

int main(void)
{
	have_battery = false;
	have_usb = false;
	PWR_SW = false;
	charging = false;
	// initialization calls
	i2cInit();
	i2cSetLocalDeviceAddr(I2C_ADDR, true);
	i2cSetSlaveReceiveHandler(i2cSlaveReceiveService);
	i2cSetSlaveTransmitHandler(i2cSlaveTransmitService);

 	/*Set baud rate */
 	UBRR0H = (unsigned char)((MYUBRR)>>8);
 	UBRR0L = (unsigned char) MYUBRR;
 	/* Enable receiver and transmitter   */
 	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
 	/* Frame format: 8data, No parity, 1stop bit */
 	UCSR0C = (3<<UCSZ00);
 	
	 
	// therm as input
	DDRC &= ~(1<<DDC3); 
	PORTC &= ~(1<<DDC3); 
	
	// LED 4 as OUTPUT
	DDRB |= (1<<DDB0); 
	// Charge pin as OUTPUT
	DDRB |= (1<<DDB1);
	PORTB |= (1<<DDB1);
	// Relay puns as OUTPUTS
	DDRD |= (1<<DDD5);
	DDRD |= (1<<DDD6);
	DDRD |= (1<<DDD7);
	PORTD &= ~(1<<DDB5);
	PORTD &= ~(1<<DDB6);
	PORTD &= ~(1<<DDB7);
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
		battery_voltage = ((double) ADC)/1024.0*256; // Read ADC
		
		have_battery = (battery_voltage/256.0*8.9) > 7;
		PWR_SW = ~(PINB >> DDB2)&0x01;		 
		
		if(have_usb) {
			PORTB &= ~(1<<PORTB0);
			_delay_ms(100);
			PORTB |= (1<<PORTB0);
			_delay_ms(500);
			
			if(!charging) startCharge();
		} else {
			if(charging) stopCharge();
			 if (!have_battery) {
			PORTB &= ~(1<<PORTB0);
			_delay_ms(50);
			PORTB |= (1<<PORTB0);
			_delay_ms(50);
			} else {
				PORTB |= (1<<PORTB0);
				_delay_ms(500);
			}
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
