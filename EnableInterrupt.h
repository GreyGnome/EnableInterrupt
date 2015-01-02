#include <Arduino.h>
// in vim, :set ts=2 sts=2 sw=2 et

// EnableInterrupt, a library by GreyGnome.  Copyright 2014 by Michael Schwager.

/*
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/ 

// Many definitions in /usr/avr/include/avr/io.h

#define PINCHANGEINTERRUPT 0x80
#define SLOWINTERRUPT      0x80

/* Arduino pin to ATmega port translaton is found doing digital_pin_to_port_PGM[] */
/* Arduino pin to PCMSKx bitmask is found by doing digital_pin_to_bit_mask_PGM[] */

#define digital_pin_to_PCMSK_bitmask digital_pin_to_bit_mask_PGM
volatile uint8_t *pcmsk;

const uint8_t PROGMEM digital_pin_to_port_bit_number_PGM[] = {
  0, // 0 == port D, 0
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  0, // 8 == port B, 0
  1,
  2,
  3,
  4,
  5,
  0, // 14 == port C, 0
  1,
  2,
  3,
  4,
  5,
};

//we can declare all of these functionPointerArrays for the PORTS without gobbling memory,
//I think, because the compiler won't create what the program doesn't use. TBD. -MIKE
void (*functionPointerArrayPORTB[6])(void); // 2 of the interrupts are unsupported on Arduino UNO.
void (*functionPointerArrayPORTC[6])(void); // 1 of the interrupts are used as RESET on Arduino UNO.
void (*functionPointerArrayPORTD[8])(void);

// For Pin Change Interrupts; since we're duplicating FALLING and RISING in software,
// we have to know how we were defined.
static volatile uint8_t risingPinsPORTB=0;
static volatile uint8_t fallingPinsPORTB=0;
static volatile uint8_t risingPinsPORTC=0;
static volatile uint8_t fallingPinsPORTC=0;
static volatile uint8_t risingPinsPORTD=0;
static volatile uint8_t fallingPinsPORTD=0;

// Arduino.h has these, but the block is surrounded by #ifdef ARDUINO_MAIN
#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 10
#define PK 11
#define PL 12
#define TOTAL_PORTS 13 // The 12 above, plus 0 which is not used.

// for the saved state of the ports
static volatile uint8_t portSnapshot[TOTAL_PORTS];
// which pins on the port are defined rising
static volatile uint8_t portRisingPins[TOTAL_PORTS];
// which pins on the port are defined falling. If a pin is marked CHANGE, it will be in both arrays.
static volatile uint8_t portFallingPins[TOTAL_PORTS];

// current state of a port inside the interrupt handler.
volatile uint8_t current;

// From /usr/share/arduino/hardware/arduino/cores/robot/Arduino.h
// #define CHANGE 1
// #define FALLING 2
// #define RISING 3

void enableInterrupt(uint8_t interruptDesignator, void (*userFunction)(void), uint8_t mode) {
  uint8_t arduinoPin;
  uint8_t EICRAvalue;
  uint8_t EIMSKvalue=1;
  uint8_t portNumber;
  uint8_t portMask;

  arduinoPin=interruptDesignator & ~PINCHANGEINTERRUPT;

  portMask=digital_pin_to_bit_mask_PGM[arduinoPin];

  // Pin Change Interrupts
	if ( (interruptDesignator && PINCHANGEINTERRUPT) || (arduinoPin != 2 && arduinoPin != 3) ) {
    portNumber=digital_pin_to_port_PGM[arduinoPin];

    // save the mode
    if ((mode == RISING) || (mode == CHANGE)) {
      if (portNumber==PB)
        risingPinsPORTB |= portMask;
      if (portNumber==PC)
        risingPinsPORTC |= portMask;
      if (portNumber==PD)
        risingPinsPORTD |= portMask;
    }
    if ((mode == FALLING) || (mode == CHANGE)) {
      if (portNumber==PB)
        fallingPinsPORTB |= portMask;
      if (portNumber==PC)
        fallingPinsPORTC |= portMask;
      if (portNumber==PD)
        fallingPinsPORTD |= portMask;
    }
    interruptModes[arduinoPin]=mode;

    // set the appropriate PCICR bit
    PCICR |= digitalPinToPCICRbit(arduinoPin);
    // save the initial value of the port
    portSnapshot[portNumber]=*port_to_input_PGM[portNumber];
    // assign the function to be run in the ISR
    if (portNumber==PB) 
      functionPointerArrayPORTB[digital_pin_to_port_bit_number_PGM[arduinoPin]] = userFuncton;
    if (portNumber==PC) 
      functionPointerArrayPORTC[digital_pin_to_port_bit_number_PGM[arduinoPin]] = userFuncton;
    if (portNumber==PD) 
      functionPointerArrayPORTD[digital_pin_to_port_bit_number_PGM[arduinoPin]] = userFuncton;
    // set the appropriate bit on the appropriate PCMSK register
    pcmsk=digitalPinToPCMSK(arduinoPin);        // appropriate PCMSK register
    *pcmsk |= portMask;  // appropriate bit
    // digitalPinToBitMask(P) calls
    // digital_pin_to_bit_mask_PGM[]: Given the Arduino pin, returns the bit associated with its port.

    // With the exception of the Global Interrupt Enable bit in SREG, interrupts on the arduinoPin
    // are now ready. This bit may have already been set on a previous enable, so it's important
    // to take note of the order in which things were done, above.

  // *******************
  // External Interrupts
  } else {
    EICRAvalue=mode;
    // MUST BE FIXED. THIS IS LIKELY UNNCESSESARY. -MIKE
    interruptFunctionPointerArray[arduinoPin] = userFunction;

    if (arduinoPin == 3) {
      EICRAvalue << 2;
      EIMSKvalue << 1;
    }
    EIMSK|=EIMSKvalue;
    EICRA|=EICRAvalue;
  }
  SREG |= (1 << SREG_I); // from /usr/avr/include/avr/common.h
}

// NOTE: PCINT1_vect is different on different chips.
#define PORTB_VECT PCINT1_vect
ISR(PORTB_VECT) {
  uint8_t changedPins;
  uint8_t interruptMask;
  uint8_t i;

  // current:      0b 0000 0000
  // portSnapshot: 0b 0000 0000
  // Given: Changed pin on the port.
  //        Port
  //        Pin of interest on the port.
  //        Mode chosen for the pin on the port.
  // Find:  If this change is of interest, we run the interrupt.
  interruptMask = PCMSK0;
	current = portInputRegister(PB);
  changedPins=(portSnapshot[PB] ^ current) & ((risingPinsPORTB & current) | (fallingPinsPORTB & ~current));
  portSnapshot[PB] =  current;
  
  for (i=0; i < 6; i++) {
    if (0x01 & interruptMask & changedPins) {
      (*functionPointerArrayPORTB[i])();
      return();
    }
    interruptMask >> 1;
  }
}

