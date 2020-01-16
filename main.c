/********************************************
 *
 *  Name: Gabe Dalessandro
 *  Email: gdalessa@usc.edu
 *  Section: 12:30PM Friday
 *  Assignment: Final Project: Speed
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "lcd.h"


//Distance between Sensors
#define DISTANCE 508 //5.08cm *100


//setting up the RS-232 Communication
#define FOSC 16000000           // Clock frequency
#define BAUD 9600               // Baud rate used
#define MYUBRR (FOSC/16/BAUD-1) // Value for UBRR0 register


#define ADC_CHAN 0      // Buttons use ADC channel 0

//TIMER 1 (16 bit timer)
volatile char complete = 0;
volatile long measuredSpeed = 0;
volatile double compareSpeed = 0;
volatile char maxSpeedReached = 0;


//TIMER 0 (8 bit timer)
volatile char timer0Buf[16];
volatile char alarmFlag = 0;
volatile int timer0Counter = 0;
volatile char alarmOn = 0;

//ROTARY ENCODER
volatile unsigned char new_state, old_state;
extern volatile unsigned char changed = 0;  // Flag for state change
volatile unsigned char a, b;
volatile unsigned char pn1;
volatile int maxSpeed = 30;
//volatile char maxSpeedBuf[17];
//volatile unsigned char maxSpdReached = 0;


// SERIAL COMMUNICATIONS functions and variables
void serial_init(unsigned short);
void serial_stringout(char *);
void serial_txchar(char);

volatile int receivedFlag = 0;
volatile char receivedMessage[16];
volatile int receivedIndex = 0;

int main(void) {


	maxSpeed = eeprom_read_byte((void*) 100);

    // Initialize the LCD, ADC, Interrupts and serial modules  
    lcd_init();  // Initialize the LCD 
    serial_init(MYUBRR); //initialize the serial communication
    
    sei();	//Enable the interrupts
   	PCICR |= (1 << PCIE2);
   	PCICR |= (1 << PCIE1);
	PCMSK2 |= (1 << PCINT19) | (1 << PCINT18); //interrupts for PD3 and PD2
											   //used for START and FINISH pins
	PCMSK1 |= (1 << PCINT11) | (1 << PCINT10); //interrupts for PC3 and PC2
											   //used for ROTARY encoder
	//serial communication interrupts
	UCSR0B |= (1 << RXCIE0);    // Enable receiver interrupts


	//CLOCK 1 (16-bit timer) and PRESCALER value
  	TCCR1B |= (1 << WGM12); // start over at 0 once the counter reaches your desired value
  	TIMSK1 |= (1 << OCIE1A); //enabling the 16 bit timer	
  	//Prescalers (16 bit timer)
        //16mHz * 4 = 64,000,000 cycles. 
        //64,000,000 / 1024 = 62500			1 sec = a count of 15625
    OCR1A = 62500; //will determine when to stop and generate an interrupt



  	//CLOCK 0 (8 bit timer) and PRESCALER value
  	TCCR0B |= (1 << WGM02);
  	TIMSK0 |= (1 << OCIE0A);
  	//Prescalers (8 bit timer) 2^8 = 256
  		//Desired Clock Cycles = 16mHz * .00001 = 1,600
  		//Prescaler = CS01
  	OCR0A = 200;



  	//Initialize the PORTS
  	DDRC |= (1 << PC1);  //for start LED - set as output
  	DDRC |= (1 << PC4);  //for BUZZER - set as output
  		//Pull-up resistors for the rotary encoder
  	PORTC |= (1 << PC3) | (1 << PC2);


    // Show the splash screen
    lcd_writecommand(1);
    lcd_moveto(0,0);
    lcd_stringout("Speed Trap");
    lcd_moveto(1,0);
    lcd_stringout("Gabe Dalessandro");
    _delay_ms(1000);
    lcd_writecommand(1);



init_encoder();


	char maxSpeedBuf[17];
	char timerBuf[17];
	char speedBuf[17];


    while (1) {                 // Loop forever
    	
    	if(changed == 1) {
    		changed = 0;

	    	lcd_moveto(0,0);
		    lcd_stringout("                 ");
		    lcd_moveto(1,0);
		    lcd_stringout("MaxSpeed: ");
		    lcd_moveto(1,10);

		    if(maxSpeed < 1) {
		        maxSpeed = 1;
		    }
			else if(maxSpeed > 99) {
		        maxSpeed = 99;
			}

			
		    snprintf(maxSpeedBuf, 17, "%2d", maxSpeed);
		    lcd_stringout(maxSpeedBuf);
		    lcd_stringout("cm/s");


		}
		if(complete) {
			//calculating the speed in mm/s
			long timerVal = TCNT1;
			long dist = 508;
	        long numerator = dist * 15625;
	        long denominator = 10 * (long)TCNT1;
	        measuredSpeed = numerator/denominator;
	        long speed1s = measuredSpeed/10;
	        long speedTenths = measuredSpeed%10;
	        compareSpeed = measuredSpeed/10;
	        //snprintf(speedBuf, 17, "%ld", numerator);
	        //snprintf(speedBuf, 17, "%ld", denominator);
	        //snprintf(timerBuf, 17, "%7u", TCNT1)
	        //double d_testSpeed = numerator/denominator;
			//snprintf(speedBuf, 17, "%f", d_testSpeed);


			//calculating elaptime in ms
			int32_t milliseconds = timerVal * 1000;
			milliseconds /= (long)15625;
			snprintf(timerBuf, 17, "%ld ms", milliseconds);


	        //prints the speed in cm/s
			snprintf(speedBuf, 17, "%ld.%ld cm/s", speed1s, speedTenths);
	        //snprintf(speedBuf, 17, "%ld", measuredSpeed);

			if(compareSpeed > maxSpeed) {
				maxSpeedReached = 1;
			}
			else {
		        lcd_writecommand(1);
		        lcd_moveto(0,0);
		        lcd_stringout("TIME:  ");
		        lcd_stringout(timerBuf);
		        lcd_moveto(1,0);
		        lcd_stringout("SPEED: ");
		        lcd_stringout(speedBuf);

			}


			if(maxSpeedReached == 1) {
				TCCR0B |= (1 << CS01);


				measuredSpeed = 0;
				compareSpeed = 0;
				maxSpeedReached = 0;

				lcd_writecommand(1);
				lcd_moveto(0,3);
				lcd_stringout("MAX  SPEED");
				lcd_moveto(1,4);
				lcd_stringout("REACHED!");

			}

			complete = 0;
		} //if (complete)

    } // while (1)   
} //int main(void)



//used to sound the alarm
ISR(TIMER0_COMPA_vect)
{
	timer0Counter++;

	if(timer0Counter == 4000) {
		TCCR0B &= ~(1 << CS01);
		timer0Counter = 0;
	}

	if(!alarmOn) { 
		PORTC |= (1 << PC4);
		alarmOn = 1;
	}
	else if (alarmOn) {
		PORTC &= ~(1 << PC4);
		alarmOn = 0;
	}

}



//interrupt used for START and FINISH pin
ISR(PCINT2_vect) {
        

    //if PT is covered, output == 0
    //if PT sees light, output == 1

    //START was triggered
    if( (PIND & (1 << PD3)) == 0) {
        lcd_writecommand(1);
        lcd_moveto(0,0);
        lcd_stringout("START");

        //starts the timer
        //TCCR1B &= ~(1 << TCNT1); //clear the TCNT1
        TCNT1 = 0;
        TCCR1B |= (1 << CS12);
        TCCR1B |= (1 << CS10);


        //turn ON the LED
        PORTC |= (1 << PC1);

    }


    //FINISH was triggered
    if( (PIND & (1 << PD2)) == 0) {

        //turns the clock off
        TCCR1B &= ~(1 << CS12);
        TCCR1B &= ~(1 << CS10);

        complete = 1;


		//turn OFF the LED
        PORTC &= ~(1 << PC1);
	

    }

}


// ISR(PCINT1_vect) {
// //used for rotary encoder
// 	pn1 = PINC;	//saves all 8 bits in that moment of time
//     //a is LSB
//     a = pn1 & (1 << PC3);
//     //b is MSB
//     b = pn1 & (1 << PC2);

// 	//state 00
// 	if (old_state == 0) {

// 	    // Handle A and B inputs for state 0
// 	    if (a) { 
// 	    	new_state = 1;
// 	    	maxSpeed++;
// 	    }

// 	    else if (b){
// 	    	new_state = 2;
// 	    	maxSpeed--;
// 	    }

// 	}
// 	//state 01
// 	else if (old_state == 1) {

// 	    // Handle A and B inputs for state 1
// 	    if (b) {
// 	    	new_state = 3;
// 	    	maxSpeed++;
// 	    }

// 	    else if (!a) {
// 	    	new_state = 0;
// 	    	maxSpeed--;
// 	    }

// 	}

// 	//state 10
// 	else if (old_state == 2) {
// 	    // Handle A and B inputs for state 3
// 	    if (!b) {
// 	    	new_state = 0;
// 	    	maxSpeed++;

// 	    }

// 	    else if (a) {
// 	    	new_state = 3;
// 	    	maxSpeed--;
// 	    }

// 	}

// 	//state 11
// 	else {   // old_state = 3
// 	    // Handle A and B inputs for state 2
// 	    if (!a) {
// 	    	new_state = 2;
// 	    	maxSpeed++;
// 	    }

// 	    else if (!b) {
// 	    	new_state = 1;
// 	    	maxSpeed--;
// 	    }
// 	}



// 	// If state changed, update the value of old_state,
// 	// and set a flag that the state has changed.
// 	if (new_state != old_state) {
// 	    changed = 1;
// 	    old_state = new_state;
	    
// 	}

// 	eeprom_update_byte((void*) 100, maxSpeed);
// }





//Stops the program when a 4 seconds have passed
ISR(TIMER1_COMPA_vect) {
	//turns the clock off
	TCCR1B &= ~(1 << CS12);
	TCCR1B &= ~(1 << CS10);

	//turn OFF the LED
    PORTC &= ~(1 << PC1);

	lcd_writecommand(1);
	lcd_moveto(0,0);
	lcd_stringout("4sec Max Reached");


}


void serial_init(unsigned short ubrr_value)
{

    // Set up USART0 registers
    UBRR0 = ubrr_value;
    UCSR0C = (3 << UCSZ00);               // Async., no parity,
                                          // 1 stop bit, 8 data bits
    UCSR0B |= (1 << TXEN0 | 1 << RXEN0);  // Enable RX and TX

    // Enable tri-state buffer
    DDRD |= (1 << PD5); //enable line
    PORTD &= ~(1 << PD5);

}


void serial_txchar(char ch)
{
    while ((UCSR0A & (1<<UDRE0)) == 0);
    UDR0 = ch;
}

void serial_stringout(char *s)
{

    // Call serial_txchar in loop to send a string
    signed int i = 0;
    while(s[i] != '\0') {
        serial_txchar(s[i]);
        i++;
    }
    // (Very similar to lcd_stringout in lcd.c)

}

ISR(USART_RX_vect)
{

    // Handle received character
    char ch;

    ch = UDR0;                  // Get the received charater

    // Store in buffer
    receivedMessage[receivedIndex] = ch;
    receivedIndex++;

    // If message complete, set flag
    if(receivedIndex == 16) {
        receivedIndex++;
        receivedMessage[receivedIndex] = '\0';
        receivedFlag = 1;
    }


}



