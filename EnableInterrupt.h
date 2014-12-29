#include <Arduino.h>
// in vim, :set ts=2 sts=2 sw=2 et

// enableInterrupt, a library by Mike Schwager aka GreyGnome.
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

static volatile void *interruptFunctionPointerArray[NUM_DIGITAL_PINS];
// For Pin Change Interrupts; since we're duplicating FALLING and RISING in software,
// we have to know how we were defined.
static volatile int interruptModes[NUM_DIGITAL_PINS];

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

// for the state of the ports
static volatile uint8_t portSnapshot[sizeof(port_to_input_PGM)];

// From /usr/share/arduino/hardware/arduino/cores/robot/Arduino.h
// #define CHANGE 1
// #define FALLING 2
// #define RISING 3

void enableInterrupt(uint8_t interruptDesignator, void (*userFunction)(void), uint8_t mode) {
  uint8_t arduinoPin;
  uint8_t EICRAvalue;
  uint8_t EIMSKvalue=1;
  uint8_t portNum;

  arduinoPin=interruptDesignator & ~PINCHANGEINTERRUPT;

  // Pin Change Interrupts
	if ( (interruptDesignator && PINCHANGEINTERRUPT) || (arduinoPin != 2 && arduinoPin != 3) ) {
    interruptModes[arduinoPin]=mode;
    // assign the function to be run in the ISR
    interruptFunctionPointerArray[arduinoPin] = userFuncton;
    // set the appropriate PCICR bit
    PCICR |= digitalPinToPCICRbit(arduinoPin);
    // save the initial value of the port
    portNum=digital_pin_to_port_PGM[arduinoPin];
    portSnapshot[portNum]=*port_to_input_PGM[portNum];
    // set the appropriate bit on the appropriate PCMSK register
    pcmsk=digitalPinToPCMSK(arduinoPin);        // appropriate register
    *pcmsk |= digital_pin_to_bit_mask_PGM[arduinoPin];  // appropriate bit
    // With the exception of the Global Interrupt Enable bit in SREG, interrupts on the arduinoPin
    // are now ready. This bit may have already been set on a previous enable, so it's important
    // to take note of the order in which things were done, above.
  } else {
    EICRAvalue=mode;
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

void 
