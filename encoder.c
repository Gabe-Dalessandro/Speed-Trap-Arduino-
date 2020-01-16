#include "encoder.h"



extern volatile char changed;


init_encoder() {
    pn1 = PINC;	//saves all 8 bits in that moment of time
    //a is LSB
    a = pn1 & (1 << PC3);
    //b is MSB
    b = pn1 & (1 << PC2);

    if (!b && !a) //00
	old_state = 0;
    else if (!b && a) //01
	old_state = 1;
    else if (b && !a) //10
	old_state = 2;
    else			   //11
	old_state = 3;

    new_state = old_state;
}

ISR(PCINT1_vect) {
//used for rotary encoder
	pn1 = PINC;	//saves all 8 bits in that moment of time
    //a is LSB
    a = pn1 & (1 << PC3);
    //b is MSB
    b = pn1 & (1 << PC2);

	//state 00
	if (old_state == 0) {

	    // Handle A and B inputs for state 0
	    if (a) { 
	    	new_state = 1;
	    	maxSpeed++;
	    }

	    else if (b){
	    	new_state = 2;
	    	maxSpeed--;
	    }

	}
	//state 01
	else if (old_state == 1) {

	    // Handle A and B inputs for state 1
	    if (b) {
	    	new_state = 3;
	    	maxSpeed++;
	    }

	    else if (!a) {
	    	new_state = 0;
	    	maxSpeed--;
	    }

	}

	//state 10
	else if (old_state == 2) {
	    // Handle A and B inputs for state 3
	    if (!b) {
	    	new_state = 0;
	    	maxSpeed++;

	    }

	    else if (a) {
	    	new_state = 3;
	    	maxSpeed--;
	    }

	}

	//state 11
	else {   // old_state = 3
	    // Handle A and B inputs for state 2
	    if (!a) {
	    	new_state = 2;
	    	maxSpeed++;
	    }

	    else if (!b) {
	    	new_state = 1;
	    	maxSpeed--;
	    }
	}



	// If state changed, update the value of old_state,
	// and set a flag that the state has changed.
	if (new_state != old_state) {
	    changed = 1;
	    old_state = new_state;
	    
	}

	eeprom_update_byte((void*) 100, maxSpeed);
}


