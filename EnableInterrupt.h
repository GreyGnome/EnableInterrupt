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
#define SLOWINTERRUPT      0x80 // same thing


void enableInterrupt(uint8_t interruptDesignator, void (*userFunction)(void), uint8_t mode);

// Example: printPSTR("This is a nice long string that takes no memory");
#define printPSTR(x) SerialPrint_P(PSTR(x))
void SerialPrint_P(PGM_P str) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) Serial.write(c);
}

/* Arduino pin to ATmega port translaton is found doing digital_pin_to_port_PGM[] */
/* Arduino pin to PCMSKx bitmask is found by doing digital_pin_to_bit_mask_PGM[] */
volatile uint8_t *pcmsk;

volatile const uint8_t PROGMEM digital_pin_to_port_bit_number_PGM[] = {
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


typedef void (*interruptFunctionType)(void);
interruptFunctionType functionPointerArrayEXTERNAL[2];
interruptFunctionType functionPointerArrayPORTB[6]; // 2 of the interrupts are unsupported on Arduino UNO.
interruptFunctionType functionPointerArrayPORTC[6]; // 1 of the interrupts are used as RESET on Arduino UNO.
interruptFunctionType functionPointerArrayPORTD[8];

// For Pin Change Interrupts; since we're duplicating FALLING and RISING in software,
// we have to know how we were defined.
volatile uint8_t risingPinsPORTB=0;
volatile uint8_t fallingPinsPORTB=0;
volatile uint8_t risingPinsPORTC=0;
volatile uint8_t fallingPinsPORTC=0;
volatile uint8_t risingPinsPORTD=0;
volatile uint8_t fallingPinsPORTD=0;

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
static volatile uint8_t portSnapshotB;
static volatile uint8_t portSnapshotC;
static volatile uint8_t portSnapshotD;
// which pins on the port are defined rising
static volatile uint8_t portRisingPins[TOTAL_PORTS];
// which pins on the port are defined falling. If a pin is marked CHANGE, it will be in both arrays.
static volatile uint8_t portFallingPins[TOTAL_PORTS];

// current state of a port inside the interrupt handler.
//volatile uint8_t current;

// From /usr/share/arduino/hardware/arduino/cores/robot/Arduino.h
// #define CHANGE 1
// #define FALLING 2
// #define RISING 3

// "interruptDesignator" is simply the Arduino pin optionally OR'ed with
// PINCHANGEINTERRUPT (== 0x80)

void enableInterrupt(uint8_t interruptDesignator, interruptFunctionType userFunction, uint8_t mode) {
  uint8_t arduinoPin;
  uint8_t EICRAvalue;
  uint8_t portNumber;
  uint8_t portMask;
  uint8_t origSREG; // to save for interrupts

  arduinoPin=interruptDesignator & ~PINCHANGEINTERRUPT;
  printPSTR("Arduino pin is "); Serial.println(arduinoPin, DEC); // OK-MIKE

  // Pin Change Interrupts
  // Arduino pin is 18
  // _BV(4) is 0x10
  // portMask is 0x10
  // portNumber is 3
  // Port C, rising pins
  // Port C, falling pins
  // PCICR bit: 0x1
  // Inital value of port: 0x10
  // function pointer array entry number: 0x8
  // 
	if ( (interruptDesignator && PINCHANGEINTERRUPT) || (arduinoPin != 2 && arduinoPin != 3) ) {
    portMask=pgm_read_byte(&digital_pin_to_bit_mask_PGM[arduinoPin]); // OK-MIKE
    printPSTR("portMask is 0x"); Serial.println(portMask, HEX);    // OK-MIKE

    portNumber=pgm_read_byte(&digital_pin_to_port_PGM[arduinoPin]);

    printPSTR("portNumber is "); Serial.println(portNumber, HEX);  // OK-MIKE

    // save the mode
    if ((mode == RISING) || (mode == CHANGE)) {
      printPSTR("Mode is change\r\n");
      if (portNumber==PB) {
        risingPinsPORTB |= portMask;
        PCMSK0 |= portMask;
      }
      if (portNumber==PC) {
        risingPinsPORTC |= portMask;
        PCMSK1 |= portMask;
        // save the initial value of the port
        printPSTR("Port C, rising pins 0x"); // OK-MIKE
        Serial.println(risingPinsPORTC, HEX);
        printPSTR("Inital value of port: 0x");
        Serial.println(*portInputRegister(portNumber), HEX);
      }
      if (portNumber==PD) {
        risingPinsPORTD |= portMask;
        PCMSK2 |= portMask;
      }
    }
    if ((mode == FALLING) || (mode == CHANGE)) {
      printPSTR("Mode is change\r\n");
      if (portNumber==PB) {
        fallingPinsPORTB |= portMask;
        PCMSK0 |= portMask;
      }
      if (portNumber==PC) {
        fallingPinsPORTC |= portMask;
        printPSTR("Port C, falling pins 0x"); // OK-MIKE
        Serial.println(fallingPinsPORTC, HEX);
        PCMSK1 |= portMask;
        printPSTR("PCMSK1 is 0x"); Serial.println(PCMSK1, HEX);
      }
      if (portNumber==PD) {
        fallingPinsPORTD |= portMask;
        PCMSK2 |= portMask;
      }
    }

    // set the appropriate PCICR bit
    printPSTR("PCICR bit: 0x");
    Serial.println(digitalPinToPCICRbit(arduinoPin), HEX);
    PCICR |= (1 << digitalPinToPCICRbit(arduinoPin)); // OK-MIKE
    // assign the function to be run in the ISR
    if (portNumber==PB) {
      functionPointerArrayPORTB[pgm_read_byte(&digital_pin_to_port_bit_number_PGM[arduinoPin])] = userFunction;
      portSnapshotB=*portInputRegister(portNumber);
    }
    if (portNumber==PC) {
      functionPointerArrayPORTC[pgm_read_byte(&digital_pin_to_port_bit_number_PGM[arduinoPin])] = userFunction;
      //Serial.print("function pointer array entry number: 0x");
      //Serial.println(pgm_read_byte(&digital_pin_to_port_bit_number_PGM[arduinoPin]), HEX);
      portSnapshotC=*portInputRegister(portNumber); // OK-MIKE
    }
    if (portNumber==PD) {
      functionPointerArrayPORTD[pgm_read_byte(&digital_pin_to_port_bit_number_PGM[arduinoPin])] = userFunction;
      portSnapshotD=*portInputRegister(portNumber); // OK-MIKE
    }
    // set the appropriate bit on the appropriate PCMSK register
    pcmsk=digitalPinToPCMSK(arduinoPin);        // appropriate PCMSK register
    *pcmsk |= portMask;  // appropriate bit, e.g. this will become PCMSK1 |= portMask;

    // digitalPinToBitMask(P) calls
    // digital_pin_to_bit_mask_PGM[]: Given the Arduino pin, returns the bit associated with its port.

    // With the exception of the Global Interrupt Enable bit in SREG, interrupts on the arduinoPin
    // are now ready. This bit may have already been set on a previous enable, so it's important
    // to take note of the order in which things were done, above.

  // *******************
  // External Interrupts
  } else {
    EICRAvalue=mode;
    origSREG = SREG;
    cli(); // no interrupts while we're setting up an interrupt.
    if (arduinoPin == 3) {
      functionPointerArrayEXTERNAL[1] = userFunction;
      EICRA=EICRA & 0b11110011;
      EICRA|=EICRAvalue << 2;
      EIMSK|=0x02;
    } else {
      functionPointerArrayEXTERNAL[0] = userFunction;
      EICRA|=EICRAvalue;
      EIMSK|=0x02;
    }
    SREG=origSREG;
  }
  SREG |= (1 << SREG_I); // from /usr/avr/include/avr/common.h
}

ISR(INT0_vect) {
  (*functionPointerArrayEXTERNAL[0])();
}

ISR(INT1_vect) {
  (*functionPointerArrayEXTERNAL[1])();
}


// NOTE: PCINT1_vect is different on different chips.
#define PORTB_VECT PCINT0_vect
#define PORTC_VECT PCINT1_vect

/*
// Future upgrade: Implement in assembly, using the ISR_NAKED attribute. See /usr/avr/include/avr/interrupt.h
ISR(PORTB_VECT, ISR_NAKED) {
  uint8_t changedPins;
  uint8_t interruptMask;
  uint8_t i;

  interruptFunction();

	current = *portInputRegister(PB);
  changedPins=(portSnapshot[PB] ^ current) & ((risingPinsPORTB & current) | (fallingPinsPORTB & ~current));
  portSnapshot[PB] =  current;
  if (changedPins == 0) return; // get out quickly if not interested.

  interruptMask = PCMSK0;
  i=0;
  while (1) {
    if (0x01 & interruptMask & changedPins) {
      (*functionPointerArrayPORTB[i])();
    }
    interruptMask=interruptMask >> 1;
    if (interruptMask == 0) return;
    i++;
  }
}
*/

volatile uint8_t changedPins=0;
volatile uint8_t functionCalled=0;
volatile uint16_t interruptsCalled=0;
volatile uint8_t risingPins=0;
volatile uint8_t fallingPins=0;

// Future upgrade: Implement in assembly, using the ISR_NAKED attribute. See /usr/avr/include/avr/interrupt.h
ISR(PORTC_VECT, ISR_NAKED) {
  uint8_t current;
  uint8_t i;
  uint8_t interruptMask;

//  uint8_t led_on, led_off;         // DEBUG
//  led_on=0b00100000; led_off=0b0;

  // BEGASM This: 
	//current = PINC; // PortC Input.
  // is the same as this:
  asm volatile("\t"
  "push %0" "\t\n\t"
  "in %0,%1" "\t\n\t"
  : "=&r" (current)
  : "I" (_SFR_IO_ADDR(PINC))
  );
  // ENDASM ...End of the same.

  // Arrgh... dump of asm shows that 'current' == r25 (from the in, above). So we push all other
  // registers. Is there no mechanism to say 'push all registers except X'? I doubt it. So I just
  // have to check it here, after every time we modify this ISR...
  asm volatile(
  "push r1" "\n\t"
  "push r0" "\n\t"
  // in 0x3f saves SREG... then it's pushed onto the stack.
  "in r0, __SREG__" "\n\t" // 0x3f
  "push r0" "\n\t"
  "eor r1, r1" "\n\t"
  "push r18" "\n\t"
  "push r19" "\n\t"
  "push r20" "\n\t"
  "push r21" "\n\t"
  "push r22" "\n\t"
  "push r23" "\n\t"
  "push r24" "\n\t"
  "push r26" "\n\t"
  "push r27" "\n\t"
  "push r28" "\n\t"
  "push r29" "\n\t"
  "push r30" "\n\t"
  "push r31" "\n\t"
  :
  :); 

  //interruptsCalled++; // DEBUG
  changedPins=(portSnapshotC ^ current) & ((risingPinsPORTC & current) | (fallingPinsPORTC & ~current));
  portSnapshotC = current;
  interruptMask = PCMSK1 & changedPins; // We are only interested in pins that were changed
                                        // AND are marked in PCMSK1.
  if (interruptMask == 0) goto exitISR; // Get out quickly if nothing we are interested in changed.
  i=0;
  while (1) {
    if (interruptMask & 0x01) {
      // PORTB=ledoff; // LOW // DEBUG
      // PORTB=ledon; // HIGH
      // PORTB=ledoff; // LOW
      // PORTB=ledon; // HIGH
      (*functionPointerArrayPORTC[i])();
    }
    interruptMask=interruptMask >> 1;
    if (interruptMask == 0) goto exitISR;
    i++;
  }
  exitISR:
  asm volatile(
  "pop r31" "\n\t"
  "pop r30" "\n\t"
  "pop r29" "\n\t"
  "pop r28" "\n\t"
  "pop r27" "\n\t"
  "pop r26" "\n\t"
  "pop r24" "\n\t"
  "pop r23" "\n\t"
  "pop r22" "\n\t"
  "pop r20" "\n\t"
  "pop r21" "\n\t"
  "pop r19" "\n\t"
  "pop r18" "\n\t"
  "pop r0" "\n\t"
  "out __SREG__, r0" "\t\n\t"
  "pop r0" "\t\n\t"
  "pop r1" "\t\n\t"
  "pop r25" "\n\t"
  "reti" "\t\n\t"
  :
  :);
}


/* ISR prefix code
push    r1
push    r0
in      r0, 0x3f        ; 63
push    r0
eor     r1, r1
push    r18
push    r19
push    r20
push    r21
push    r22
push    r23
push    r24
push    r25
push    r26
push    r27
push    r28
push    r29
push    r30
push    r31
*/
/* ISR suffix code
pop     r31
pop     r30
pop     r29
pop     r28
pop     r27
pop     r26
pop     r25
pop     r24
pop     r23
pop     r22
pop     r21
pop     r20
pop     r19
pop     r18
pop     r0
out     0x3f, r0        ; 63
pop     r0
pop     r1
reti
*/
