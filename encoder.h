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


unsigned char new_state, old_state;
extern volatile char changed;
extern volatile maxSpeed;
unsigned char a, b;
volatile unsigned char pn1;

init_encoder();
