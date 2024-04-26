/*
 * EE.ELE.250 Project.c
 *
 * Modified: 26.4.2024
 * Author : Eemeli
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <stdio.h>
#include <float.h>
#include <stdlib.h>


// Constants for the baud rate generation
#define BAUD 9600		// Baud rate
#define MYUBRR (F_CPU/16/BAUD-1) // USART Baud rate

// USART RX ring buffer constants
#define BUFFER_SIZE 8
#define BUFFER_ERROR 0
#define BUFFER_OK 1

// Variables for the ring buffer
uint8_t buffer[BUFFER_SIZE];
int16_t writep=0, readp=0, elements=0;

// Power switch interrupt variables
volatile uint8_t cPortInput; // 
volatile uint8_t sleep = 0;

// Global timer interrupt variable
volatile unsigned long timer2_millis;

// main() variables

// Previous time variables
uint32_t previousTime = 0, previousTimeADC = 0;

// Frequency variables
uint16_t scaledFrequency = 0;
int8_t frequencyOffset = 0;
uint16_t frequencySet = 0;

// ADC variables
uint16_t adcValue = 0, previousAdcValue = 0;

// USART read and parse variables
uint8_t slot = 0;
char readBuffer[8];
char newBuffer[4];

// Function prototypes
void init_power_switch_interrupt(void);
void init_millis(void);
void init_usart(void);
void init_sounder_PWM(void);
void init_led_PWM(void);

void enter_low_power_mode(void);
unsigned long millis(void);

uint8_t writebuf(uint8_t val);
uint8_t readbuf(uint8_t *val);
void zero_read_buffer(void);
void USART_Read(void);
void USART_Transmit(unsigned char data);
void USART_Print(char messageBuffer[32]);

uint16_t adc_read_pin1(void);
uint16_t scale_frequency(uint16_t input);
uint16_t limit_frequency(uint16_t input);



//Interrupts

// Power switch
ISR(PCINT1_vect)
{
	// It is not stable to enter low power modes from within interrupts
	// So instead we set a variable, which we access from main()
	sleep = 1;
}

// millis() timer
ISR(TIMER2_COMPA_vect)
{
	// Increment every time interrupt triggers (1ms)
	timer2_millis++;
}

// USART RX
ISR(USART_RX_vect) {
	// Write data from USART register to ring buffer
	writebuf(UDR0); 
}

// Register initialization
void init_power_switch_interrupt(void)
{
	DDRC &= ~(1 << PC5); // Set PC5 to input
	PCICR |= (1 << PCIE1); // Enable pin change interrupts on port C
	PCMSK1 |= (1 << PCINT13); // Enable PCINT13 (pin PC5)
}

void init_millis(void)
{
	//Using timer2
	
	TCCR2A |= (1 << WGM21); // Set timer to clear when matching ctc_match_overflow
	TCCR2B |= (1 << CS22); // Set prescaler to 64

	OCR2A = ((F_CPU / 1000UL) / 64UL); // Clear timer every 1ms with a prescaler of 64

	TIMSK2 |= (1 << OCIE2A); // Enable the compare match interrupt
}

void init_usart(void)
{
	for(int i=0;i<BUFFER_SIZE;i++)
	buffer[i]=1;
	
	zero_read_buffer();
	
	// Set baud rate, 16 bit register
	UBRR0H = (unsigned char)(MYUBRR>>8);// Set high byte by right shifting
	UBRR0L = (unsigned char)MYUBRR;	// Type cast discards the high byte
	
	// Enable receiver and transmitter
	UCSR0B |= (1<<RXEN0);
	UCSR0B |= (1<<TXEN0);
	

	// Set frame format: 8 data, 1 stop bits, no parity
	// UCSR0C defaults are just fine

	// clear pending interrupt flag
	UCSR0A &= ~(1<<RXC0);

	// enable receive complete interrupt
	UCSR0B |= (1<<RXCIE0);
}

void init_sounder_PWM(void) {
	//Using timer1
	
	// Set PWM pin (OC1B, pin PB2) as output
	DDRB |= (1 << PB2);

	// Set timer mode to fast PWM with non-inverting output (Mode 15)
	TCCR1A |= (1 << COM1B1) | (1 << WGM11) | (1 << WGM10);;
	TCCR1B |= (1 << WGM12) | (1 << WGM13);

	TCCR1B |= (1 << CS12); // Set prescaler to 256

	// Set initial frequency
	OCR1A = 1249;
	// Set initial duty cycle
	OCR1B = 0;
}

void init_led_PWM(void) {
	//Using timer0
	
	// Set PWM pin (OC0A, pin PD6) as output
	DDRD |= (1 << PD6);

	// Set timer mode to fast PWM with non-inverting output (Mode 3)
	TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B |= (1 << CS01); // Set prescaler to 1

	// Set initial duty cycle
	OCR0A = 127;
}

void enter_low_power_mode(void)	{
	// set power-down sleep mode for lowest power consumption
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
	cli(); // disable interrupts
	sleep_enable(); // set SE-bit
	sei(); // enable interrupts
	sleep_cpu(); // SLEEP-instruction
	
	// entry-point after wake-up
	sleep_disable(); // reset SE-bit
	sei(); // enable interrupts
	sleep = 0;
}

unsigned long millis (void)
{
	unsigned long millis_return;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // Ensure this cannot be disrupted
		millis_return = timer2_millis;
	}
	return millis_return;
}

// USART ring buffer
// Write element to ring buffer
uint8_t writebuf(uint8_t val) {
	/*	Places val to ring buffer, does not overwrite unread data
		Returns BUFFER_OK when write completed
		Returns BUFFER_ERROR when buffer is full
	*/
	if(elements < BUFFER_SIZE) {	// is there room in the buffer?
		buffer[writep++] = val;	// store new element
		elements++; // Increments elements to show that data has been added
	} else
	{
		return BUFFER_ERROR; // no room!
	}	
	
	if(writep >= BUFFER_SIZE) // Reset writep when buffer size is exceeded
	{
		writep = 0;
	}
	
	return BUFFER_OK;
}

// Read oldest element from ring buffer
uint8_t readbuf(uint8_t *val) {
	/*	Places oldest element from ring buffer to *val
		Returns BUFFER_OK when read completed
		Returns BUFFER_ERROR when there was nothing to read
	*/
	if(elements > 0) {
		*val = buffer[readp++]; // Read element
		elements--; // Decrements elements to allow for new data to be written
	} else
	{
		return BUFFER_ERROR;
	}
	
	if(readp >= BUFFER_SIZE) // Reset readp when buffer size is exceeded
	{
		readp=0;
	}
	
	return BUFFER_OK;
}

void zero_read_buffer(void)
{
	// Fill readBuffer with all 0's (helps with atoi)
	for (uint8_t i = 0; i<8; i++)
	{
		readBuffer[i] = '0';
	}
}

void USART_Read(void)	
{
	/*	Read and parse USART RX data
		Sets frequencyOffset or frequencySet depending on user input
		
		readBuffer works as a ring buffer which overwrites old data
	*/
	
	uint8_t a; // Byte to read
	uint8_t elementsCopy = elements; // Copy of element count
	
	//Read all available characters
	while(elementsCopy != 0)	{ // Use the copy so it has no chance to update mid loop
		readbuf(&a); // Read character
		readBuffer[slot] = a;
		
		// Parse data
		if(readBuffer[slot] == '+')	
		{
			frequencyOffset = 10;
			frequencySet = 0;
			zero_read_buffer();
		}
		else if(readBuffer[slot] == '-')	
		{
			frequencyOffset = -10;
			frequencySet = 0;
			zero_read_buffer();
		}
		else if(readBuffer[slot] == '\r')	// Enter key has been pressed (\r)
		{
			for (uint8_t i = 0; i<4; i++) // Write the last 4 characters to a new buffer
			{
				newBuffer[i] = readBuffer[(slot+i+4)%8]; // Modulo used to access lower elements
			}
			zero_read_buffer();
			
			frequencySet = atoi(newBuffer); // Convert char array to integer
		}

		slot++;
		if(slot >= 8)	{
			slot = 0;
		}
		elementsCopy = elements;
	}
}

void USART_Transmit(unsigned char data)
{
	// Transmit a single byte over USART
	while ( !( UCSR0A & (1<<UDRE0)) ){;;} // Wait until TX buffer is empty
	UDR0 = data; // Place data to TX register
}

void USART_Print(char messageBuffer[32])
{
	// Transmit a character array one character at a time
	for (uint8_t i = 0; messageBuffer[i] != '\0' || i>=31; i++) {
		USART_Transmit(messageBuffer[i]);
	}
}

uint16_t adc_read_pin1(void) { // time <1ms
	/* Sample voltage on ADC1 Pin */

	ADCSRA |= (1<<ADEN); // enable adc
	
	ADMUX = 0; // Zero out ADMUX in case it is modified elsewhere
	ADMUX	|=	(1<<REFS0); // Use AVcc as Voltage reference
	ADMUX	|=	(1<<MUX0); // Sample voltage on ADC1 pin (PC1)

	ADCSRA |= (1<<ADSC); // Start conversion
	while ( (ADCSRA & (1<<ADSC)) ); // Wait for conversion to finish
	
	ADCSRA &= ~(1<<ADEN); // disable adc
	
	return ADC; // The result is returned from the ADC result register
}

uint16_t scale_frequency(uint16_t input)
{
	float value = (float)input * 950 / 1023 + 50; // Scale adc value to given bounds of 50-1000Hz
	return (uint16_t)value; // Cast value float back to uint16_t
}

uint16_t limit_frequency(uint16_t input)
{
	if(input > 1000) // Limit frequency to 50-1000hz
	{
		return 1000;
	}else if(input < 50)
	{
		return 50;
	}
	
	return input;
}

int main(void)
{
	
	init_power_switch_interrupt();
	init_usart();
	init_millis();
	init_sounder_PWM();
	init_led_PWM();
	
	sei();
	
	while(1) {
		USART_Read();// Read and parse USART data
		
		if(millis() > previousTimeADC + 100) // Sample ADC and set frequency at 10Hz
		{
			previousTimeADC = millis(); // Record previous ADC time
			previousAdcValue = adcValue; // Record previous ADC value
			adcValue = adc_read_pin1(); // Get new ADC value
			
			// Change value with pot or with terminal
			// + or - characters change the current frequency by 10
			// 4 number characters followed by CR (Enter) allows manual setting to specific frequency
			// Move pot to go back to pot value
			// Add bounds so that noise doesn't appear as a changed value
			if(adcValue <= previousAdcValue - 10 || adcValue >= previousAdcValue + 10)
			{
				scaledFrequency = scale_frequency(adcValue); // scale frequency from 0-1023 to 50-1000
				frequencyOffset = 0;
				frequencySet = 0;
			} else
			{
				if(frequencySet == 0)	{
					scaledFrequency += frequencyOffset; // + or - character
					frequencyOffset = 0;
				}else
				{
					scaledFrequency = frequencySet; // Manually typed number
				}	
			}
		
			scaledFrequency = limit_frequency(scaledFrequency); // Frequency is limited within 50-1000Hz
		
			uint16_t pwmValue = (F_CPU / (256UL * scaledFrequency)) - 1; // Apply datasheet formula with 256 prescaler
		
			OCR1A = pwmValue; // Sounder Frequency is determined by OCR1A
			OCR1B = pwmValue/4; // Duty cycle of 25%
		
			OCR0A = adcValue / 4; // LED PWM duty cycle (10 bit value -> 8 bit value)
		}
		
		if(millis() > previousTime + 2000) // announce frequency over USART every 2 seconds
		{ 
			previousTime = millis();  // Record previous message time

			char messageBuffer[32];
			snprintf(messageBuffer, sizeof(messageBuffer), "The Frequency is: %u Hz\n\r", scaledFrequency);
			USART_Print(messageBuffer);
		}
		
		if(sleep == 1)	{ // Enter low power mode from main()
			enter_low_power_mode();
		}
	}
}

