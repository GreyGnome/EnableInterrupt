#include <Arduino.h>
// in vim, :set ts=2 sts=2 sw=2 et

// EnableInterrupt, a library by GreyGnome.  Copyright 2014-2015 by Michael Anthony Schwager.

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

/* Function Prototypes */
/* These are the only functions the end user (programmer) needs to consider. This means you! */

/* 
 * enableInterrupt- Sets up an interrupt on a selected Arduino pin.
 * 
 * Call like::
 * enableInterrupt(uint8_t interruptDesignator, void (*userFunction)(void), uint8_t mode);
 *
 * interruptDesignator: Essentially this is an Arduino pin, and if that's all you want to give
 * the function, it will work just fine. Why is it called an "interruptDesignator", then? Because
 * there's a twist: You can perform a bitwise "and" with the pin
 */
void enableInterrupt(uint8_t interruptDesignator, void (*userFunction)(void), uint8_t mode);

void disableInterrupt(uint8_t interruptDesignator);

/* End Function Prototypes */

#undef PINCHANGEINTERRUPT
#undef SLOWINTERRUPT

#define PINCHANGEINTERRUPT 0x80
#define SLOWINTERRUPT      0x80 // same thing

// Example: printPSTR("This is a nice long string that takes no static ram");
#define printPSTR(x) SerialPrint_P(PSTR(x))
void SerialPrint_P(PGM_P str) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) Serial.write(c);
} 


/* Arduino pin to ATmega port translaton is found doing digital_pin_to_port_PGM[] */
/* Arduino pin to PCMSKx bitmask is found by doing digital_pin_to_bit_mask_PGM[] */
/* ...except for PortJ, which is shifted left 1 bit in PCI1 */
volatile uint8_t *pcmsk;

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

typedef void (*interruptFunctionType)(void);

// ===========================================================================================
// CHIP SPECIFIC DATA STRUCTURES =============================================================
// ===========================================================================================

/* UNO SERIES *************************************************************************/
/* UNO SERIES *************************************************************************/
/* UNO SERIES *************************************************************************/
#if defined __AVR_ATmega168__ || defined __AVR_ATmega168A__ || defined __AVR_ATmega168P__ || \
  __AVR_ATmega168PA__ || \
  __AVR_ATmega328__ || __AVR_ATmega328P__

#define ARDUINO_328
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

// for the saved state of the ports
static volatile uint8_t portSnapshotB;
static volatile uint8_t portSnapshotC;
static volatile uint8_t portSnapshotD;

// these are defined in the avr.h files, like iom328p.h
#define PORTB_VECT PCINT0_vect
#define PORTC_VECT PCINT1_vect
#define PORTD_VECT PCINT2_vect

/* MEGA SERIES ************************************************************************/
/* MEGA SERIES ************************************************************************/
/* MEGA SERIES ************************************************************************/
#elif defined __AVR_ATmega640__ || defined __AVR_ATmega2560__ || defined __AVR_ATmega1280__ || \
  defined __AVR_ATmega1281__ || defined __AVR_ATmega2561__
#define ARDUINO_MEGA

const uint8_t PROGMEM digital_pin_to_port_bit_number_PGM[] = {
  0, // PE0  pin: 0
  1, // PE1  pin: 1
  4, // PE4  pin: 2
  5, // PE5  pin: 3
  5, // PG5  pin: 4
  3, // PE3  pin: 5
  3, // PH3  pin: 6
  4, // PH4  pin: 7
  5, // PH5  pin: 8
  6, // PH6  pin: 9
  4, // PB4  pin: 10
  5, // PB5  pin: 11
  6, // PB6  pin: 12
  7, // PB7  pin: 13
  1, // PJ1  pin: 14
  0, // PJ0  pin: 15
  1, // PH1  pin: 16
  0, // PH0  pin: 17
  3, // PD3  pin: 18
  2, // PD2  pin: 19
  1, // PD1  pin: 20
  0, // PD0  pin: 21
  0, // PA0  pin: 22
  1, // PA1  pin: 23
  2, // PA2  pin: 24
  3, // PA3  pin: 25
  4, // PA4  pin: 26
  5, // PA5  pin: 27
  6, // PA6  pin: 28
  7, // PA7  pin: 29
  7, // PC7  pin: 30
  6, // PC6  pin: 31
  5, // PC5  pin: 32
  4, // PC4  pin: 33
  3, // PC3  pin: 34
  2, // PC2  pin: 35
  1, // PC1  pin: 36
  0, // PC0  pin: 37
  7, // PD7  pin: 38
  2, // PG2  pin: 39
  1, // PG1  pin: 40
  0, // PG0  pin: 41
  7, // PL7  pin: 42
  6, // PL6  pin: 43
  5, // PL5  pin: 44
  4, // PL4  pin: 45
  3, // PL3  pin: 46
  2, // PL2  pin: 47
  1, // PL1  pin: 48
  0, // PL0  pin: 49
  3, // PB3  pin: 50
  2, // PB2  pin: 51
  1, // PB1  pin: 52
  0, // PB0  pin: 53
  0, // PF0  pin: 54
  1, // PF1  pin: 55
  2, // PF2  pin: 56
  3, // PF3  pin: 57
  4, // PF4  pin: 58
  5, // PF5  pin: 59
  6, // PF6  pin: 60
  7, // PF7  pin: 61
  0, // PK0  pin: 62
  1, // PK1  pin: 63
  2, // PK2  pin: 64
  3, // PK3  pin: 65
  4, // PK4  pin: 66
  5, // PK5  pin: 67
  6, // PK6  pin: 68
  7, // PK7  pin: 69
  2, // PJ2  pin: fake70, trick to allow software interrupts on Port J. PJ2
  3, // PJ3  pin: fake71 PJ3
  4, // PJ4  pin: fake72 PJ4
  5, // PJ5  pin: fake73 PJ5
  6, // PJ6  pin: fake74 PJ6
  6, // PE6  pin: fake75 PE6
  7, // PE7  pin: fake76 PE7
};

interruptFunctionType functionPointerArrayEXTERNAL[8];
interruptFunctionType functionPointerArrayPORTB[8];
// only 7 pins total of port J are supported as interrupts on the ATmega2560,
// and only PJ1 and 2 are supported on the Arduino MEGA.
// For PCI1 the 0th bit is PE0.   PJ2-6 are not exposed on the Arduino pins, but
// we will support them anyway. There are clones that provide them, and users may
// solder in their own connections (...go, Makers!)
interruptFunctionType functionPointerArrayPORTJ[7];
interruptFunctionType functionPointerArrayPORTK[8];

// For Pin Change Interrupts; since we're duplicating FALLING and RISING in software,
// we have to know how we were defined.
volatile uint8_t risingPinsPORTB=0;
volatile uint8_t fallingPinsPORTB=0;
volatile uint8_t risingPinsPORTJ=0;
volatile uint8_t fallingPinsPORTJ=0;
volatile uint8_t risingPinsPORTK=0;
volatile uint8_t fallingPinsPORTK=0;

// for the saved state of the ports
static volatile uint8_t portSnapshotB;
static volatile uint8_t portSnapshotJ;
static volatile uint8_t portSnapshotK;

#define PORTB_VECT PCINT0_vect
#define PORTJ_VECT PCINT1_vect
#define PORTK_VECT PCINT2_vect

/* LEONARDO ***************************************************************************/
/* LEONARDO ***************************************************************************/
/* LEONARDO ***************************************************************************/
#elif defined __AVR_ATmega32u4__ || defined __AVR_ATmega16U4__
#define ARDUINO_LEONARDO

/* To derive this list: 
   sed -n -e '1,/digital_pin_to_port_PGM/d' -e '/^}/,$d' -e '/P/p' \
       /usr/share/arduino/hardware/arduino/variants/leonardo/pins_arduino.h | \
       awk '{print "  ", $5 ", // " $5 "  pin: " $3}'
   ...then massage the output as necessary to create the below:
*/
const uint8_t PROGMEM digital_pin_to_port_bit_number_PGM[] = {
  2, // PD2  pin: D0
  3, // PD3  pin: D1
  1, // PD1  pin: D2
  0, // PD0  pin: D3
  4, // PD4  pin: D4
  6, // PC6  pin: D5
  7, // PD7  pin: D6
  6, // PE6  pin: D7
  4, // PB4  pin: D8  // we really only care about Port B, but I don't know that
  5, // PB5  pin: D9  // shortening this array and doing array index arithmetic
  6, // PB6  pin: D10 // the code any shorter.
  7, // PB7  pin: D11
  6, // PD6  pin: D12
  7, // PC7  pin: D13
  3, // PB3  pin: D14 MISO
  1, // PB1  pin: D15 SCK
  2, // PB2  pin: D16 MOSI
// There are no ports we care about after pin 16.
};

interruptFunctionType functionPointerArrayEXTERNAL[5];
interruptFunctionType functionPointerArrayPORTB[8];

// For Pin Change Interrupts; since we're duplicating FALLING and RISING in software,
// we have to know how we were defined.
volatile uint8_t risingPinsPORTB=0;
volatile uint8_t fallingPinsPORTB=0;

// for the saved state of the ports
static volatile uint8_t portSnapshotB;

#define PORTB_VECT PCINT0_vect
#endif
// ===========================================================================================
// END END END DATA STRUCTURES ===============================================================
// ===========================================================================================

// sexytime! The compiler uses the cbi and sbi instructions (which set/clear a numbered bit)!
// That's awesome- only 1 instruction per each PORTB*= instruction, below, and no registers needed!
// Cooooolll....
void interruptSaysHello() {
#ifdef ARDUINO_328
#define PINSIGNAL 13
  uint8_t led_on, led_off;         // DEBUG
  led_on=0b00100000; led_off=~led_on;  // PB5 == Arduino pin 13.
  PORTB&=led_off;
  PORTB|=led_on;
  PORTB&=led_off;
  PORTB|=led_on;
#elif defined ARDUINO_MEGA
#define PINSIGNAL 22
  uint8_t sign_on, sign_off;         // DEBUG
  sign_on=0b00000001; sign_off=~sign_on;  // PA0 == Arduino pin 22.
  PORTA&=sign_off;
  PORTA|=sign_on;
  PORTA&=sign_off;
  PORTA|=sign_on;
#endif
}


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
  uint8_t portNumber=0;
  uint8_t portMask=0;
  uint8_t portBitNumber; // when an interrupted pin is found, this will be used to choose the function.

  arduinoPin=interruptDesignator & ~PINCHANGEINTERRUPT;
  printPSTR("Arduino pin is "); Serial.println(arduinoPin, DEC); // OK-MIKE

#if defined ARDUINO_328
  if ( (interruptDesignator & PINCHANGEINTERRUPT) || (arduinoPin != 2 && arduinoPin != 3) ) {
#elif defined ARDUINO_MEGA
  // NOTE: PJ2-6 and PE6 & 7 are not exposed on the Arduino, but they are supported here
  // for software interrupts and support of non-Arduino platforms which expose more pins.
  // PJ2-6 are called pins 70-74, PE6 is pin 75, PE7 is pin 76.
  if ( (interruptDesignator & PINCHANGEINTERRUPT) || (arduinoPin != 2 && arduinoPin != 3 &&
                                                      arduinoPin != 75 && arduinoPin != 76 &&
                                                      (arduinoPin < 18 || arduinoPin > 21))
     ) {
    if (arduinoPin > 69) { // Dastardly tricks to support PortJ 2-7
      portMask=pgm_read_byte(&digital_pin_to_bit_mask_PGM[arduinoPin-6]); // Steal from PK
      portNumber=PJ;
    }
    if (arduinoPin < 70)
#else
#error Unsupported Arduino platform
#endif
    {
      portMask=pgm_read_byte(&digital_pin_to_bit_mask_PGM[arduinoPin]);
      portNumber=pgm_read_byte(&digital_pin_to_port_PGM[arduinoPin]);
    }
      ////
    printPSTR("portMask is 0x"); Serial.println(portMask, HEX);
      ////

      ////
    printPSTR("portNumber is 0x"); Serial.println(portNumber, HEX);  // OK-MIKE
      ////

    // save the mode
    if ((mode == RISING) || (mode == CHANGE)) {
      printPSTR("Mode is change\r\n");
      if (portNumber==PB) {
        risingPinsPORTB |= portMask;
      }
#if defined ARDUINO_328
      if (portNumber==PC) {
        risingPinsPORTC |= portMask;
      }
      if (portNumber==PD) {
        risingPinsPORTD |= portMask;
      }
#elif defined ARDUINO_MEGA
      if (portNumber==PJ) {
        risingPinsPORTJ |= portMask;
      ////
        printPSTR("Port J, rising pins 0x");
        Serial.println(risingPinsPORTJ, HEX);
        printPSTR("Initial value of port: 0x");
        Serial.println(*portInputRegister(portNumber), HEX);
      ////
      }
      if (portNumber==PK) {
        risingPinsPORTK |= portMask;
      }
#elif defined ARDUINO_LEONARDO
#error NOT IMPLEMENTED YET
#endif
    }
    if ((mode == FALLING) || (mode == CHANGE)) {
      printPSTR("Mode is change\r\n");
      if (portNumber==PB) {
        fallingPinsPORTB |= portMask;
      ////
        printPSTR("Port B, falling pins 0x");
        Serial.println(fallingPinsPORTB, HEX);
        printPSTR("PCMSK0 is 0x"); Serial.println(PCMSK0, HEX);
      ////
      }
#if defined ARDUINO_328
      if (portNumber==PC) {
        fallingPinsPORTC |= portMask;
      }
      if (portNumber==PD) {
        fallingPinsPORTD |= portMask;
      }
#elif defined ARDUINO_MEGA
      if (portNumber==PJ) {
        fallingPinsPORTJ |= portMask;
        printPSTR("Port J, falling pins 0x"); // OK-MIKE
        Serial.println(fallingPinsPORTJ, HEX);
        printPSTR("PCMSK1 is 0x"); Serial.println(PCMSK1, HEX);
      }
      if (portNumber==PK) {
        fallingPinsPORTK |= portMask;
      }
#elif defined ARDUINO_LEONARDO
#error NOT IMPLEMENTED YET
#endif
    }

    // assign the function to be run in the ISR
    // save the initial value of the port
    portBitNumber=pgm_read_byte(&digital_pin_to_port_bit_number_PGM[arduinoPin]);
#if defined ARDUINO_328
    if (portNumber==PC) {
      functionPointerArrayPORTC[portBitNumber] = userFunction;
      portSnapshotC=*portInputRegister(portNumber); // OK-MIKE
      pcmsk=&PCMSK1;
      PCICR |= _BV(1);
    }
    if (portNumber==PD) {
      functionPointerArrayPORTD[portBitNumber] = userFunction;
      portSnapshotD=*portInputRegister(portNumber); // OK-MIKE
      pcmsk=&PCMSK2;
      PCICR |= _BV(2);
    }
#elif defined ARDUINO_MEGA
    if (portNumber==PJ) {
      functionPointerArrayPORTJ[portBitNumber] = userFunction;
      portSnapshotJ=*portInputRegister(portNumber); // OK-MIKE
      pcmsk=&PCMSK1;
      PCICR |= _BV(1);
      portMask <<= 1; // Handle port J's oddness. PJ0 is actually 1 on PCMSK1.
    }
    if (portNumber==PK) {
      functionPointerArrayPORTK[portBitNumber] = userFunction;
      portSnapshotK=*portInputRegister(portNumber); // OK-MIKE
      pcmsk=&PCMSK2;
      PCICR |= _BV(2);
    }
#elif defined ARDUINO_LEONARDO
#error NOT IMPLEMENTED YET
#endif
    if (portNumber==PB) {
      functionPointerArrayPORTB[portBitNumber] = userFunction;
      ////
      printPSTR("function pointer array entry number: 0x");
      Serial.println(portBitNumber, HEX);
      ////
      portSnapshotB=*portInputRegister(portNumber);
      pcmsk=&PCMSK0;
      PCICR |= _BV(0);
    }
    printPSTR("We will or 0x"); Serial.print(portMask, HEX); // pcmsk was chosen above...
    printPSTR(" with PCMSKn.\r");
    *pcmsk |= portMask;  // appropriate bit, e.g. this could be PCMSK1 |= portMask;
    printPSTR("PCICR is 0x"); Serial.println(PCICR, HEX);
    printPSTR("PCMSK0 is 0x"); Serial.println(PCMSK0, HEX);
    printPSTR("PCMSK1 is 0x"); Serial.println(PCMSK1, HEX);
    printPSTR("PCMSK2 is 0x"); Serial.println(PCMSK2, HEX);

    // With the exception of the Global Interrupt Enable bit in SREG, interrupts on the arduinoPin
    // are now ready. GIE may have already been set on a previous enable, so it's important
    // to take note of the order in which things were done, above.

  // *******************
  // External Interrupts
  } else {
    Serial.print("External Interrupt chosen!");
    uint8_t EICRAvalue;
    uint8_t origSREG; // to save for interrupts
    origSREG = SREG;
    cli(); // no interrupts while we're setting up an interrupt.
#if defined ARDUINO_328
#warning EXTERNAL INTERRUPTS UNDER DEVELOPMENT
    EICRAvalue=mode;
    if (arduinoPin == 3) {
      functionPointerArrayEXTERNAL[1] = userFunction;
      EICRA=EICRA & 0b11110011;
      EICRA|=EICRAvalue << 2;
      EIMSK|=0x02;
    } else {
      functionPointerArrayEXTERNAL[0] = userFunction;
      EICRA=EICRA & 0b11111100;
      EICRA|=EICRAvalue;
      EIMSK|=0x01;
    }
#elif defined ARDUINO_MEGA
#warning EXTERNAL INTERRUPTS UNDER DEVELOPMENT
    switch (arduinoPin) {
      case 21 : // INT0
        functionPointerArrayEXTERNAL[0] = userFunction;
        EIMSK &= ~_BV(0);
        EICRA &= (~_BV(0) & ~_BV(1));
        EICRA |= mode;
        EIFR |= _BV(0);
        EIMSK |= _BV(0);
        break;
      case 20 : // INT1
        functionPointerArrayEXTERNAL[1] = userFunction;
        EIMSK &= ~_BV(1);
        EICRA &= (~_BV(2) & ~_BV(3));
        EICRA |= (mode << 2);
        EIFR |= _BV(1);
        EIMSK |= _BV(1);
        break;
      case 19 : // INT2
        functionPointerArrayEXTERNAL[2] = userFunction;
        EIMSK &= ~_BV(2);
        EICRA &= (~_BV(4) & ~_BV(5));
        EICRA |= (mode << 4);
        EIFR |= _BV(2);
        EIMSK |= _BV(2);
        break;
      case 18 : // INT3
        functionPointerArrayEXTERNAL[3] = userFunction;
        EIMSK &= ~_BV(3);
        EICRA &= (~_BV(6) & ~_BV(7));
        EICRA |= (mode << 6);
        EIFR |= _BV(3);
        EIMSK |= _BV(3);
        break;
      case  2 : // INT4
        functionPointerArrayEXTERNAL[4] = userFunction;
        EIMSK &= ~_BV(4);
        EICRB &= (~_BV(0) & ~_BV(1));
        EICRB |= mode;
        EIFR |= _BV(4);
        EIMSK |= _BV(4);
        break;
      case  3 : // INT5
        functionPointerArrayEXTERNAL[5] = userFunction;
        EIMSK &= ~_BV(5);
        EICRB &= (~_BV(2) & ~_BV(3));
        EICRB |= (mode << 2);
        EIFR |= _BV(5);
        EIMSK |= _BV(5);
        break;
      case 75 : // INT6- Fake Arduino Pin
        functionPointerArrayEXTERNAL[6] = userFunction;
        EIMSK &= ~_BV(6);
        EICRB &= (~_BV(4) & ~_BV(5));
        EICRB |= (mode << 4);
        EIFR |= _BV(6);
        EIMSK |= _BV(6);
        break;
      case 76 : // INT7- Fake Arduino Pin
        functionPointerArrayEXTERNAL[7] = userFunction;
        EIMSK &= ~_BV(7);
        EICRB &= (~_BV(6) & ~_BV(7));
        EICRB |= (mode << 6);
        EIFR |= _BV(7);
        EIMSK |= _BV(7);
        break;
    }
#elif defined ARDUINO_LEONARDO
#error NOT IMPLEMENTED YET
#endif
    SREG=origSREG;
  }
  SREG |= (1 << SREG_I); // GIE bit in SREG. From /usr/avr/include/avr/common.h
}

ISR(INT0_vect) {
#ifdef SHOWEXTERNAL
  wasExternalInterrupt++;
#endif
  (*functionPointerArrayEXTERNAL[0])();
}

ISR(INT1_vect) {
#ifdef SHOWEXTERNAL
  wasExternalInterrupt++;
#endif
  (*functionPointerArrayEXTERNAL[1])();
}

#if defined ARDUINO_MEGA
ISR(INT2_vect) {
#ifdef SHOWEXTERNAL
  wasExternalInterrupt++;
#endif
  (*functionPointerArrayEXTERNAL[2])();
}

ISR(INT3_vect) {
#ifdef SHOWEXTERNAL
  wasExternalInterrupt++;
#endif
  (*functionPointerArrayEXTERNAL[3])();
}

ISR(INT4_vect) {
#ifdef SHOWEXTERNAL
  wasExternalInterrupt++;
#endif
  (*functionPointerArrayEXTERNAL[4])();
}

ISR(INT5_vect) {
#ifdef SHOWEXTERNAL
  wasExternalInterrupt++;
#endif
  (*functionPointerArrayEXTERNAL[5])();
}

ISR(INT6_vect) {
#ifdef SHOWEXTERNAL
  wasExternalInterrupt++;
#endif
  (*functionPointerArrayEXTERNAL[6])();
}

ISR(INT7_vect) {
#ifdef SHOWEXTERNAL
  wasExternalInterrupt++;
#endif
  (*functionPointerArrayEXTERNAL[7])();
}
#endif

/*
  : "I" (_SFR_IO_ADDR(PINC)) \
*/

#define EI_ASM_PUSHIT \
   asm volatile( \
  "push r1" "\n\t" \
  "push r0" "\n\t" \
  /* in 0x3f saves SREG... then it's pushed onto the stack.*/ \
  "in r0, __SREG__" "\n\t" /* 0x3f */\
  "push r0" "\n\t" \
  "eor r1, r1" "\n\t" \
  "push r19" "\n\t" \
  "push r20" "\n\t" \
  "push r21" "\n\t" \
  "push r22" "\n\t" \
  "push r23" "\n\t" \
  "push r24" "\n\t" \
  "push r25" "\n\t" \
  "push r26" "\n\t" \
  "push r27" "\n\t" \
  "push r28" "\n\t" \
  "push r29" "\n\t" \
  "push r30" "\n\t" \
  "push r31" "\n\t" \
  : \
  :)

// We have defined "current" as "r18" in the ISR's. */
#define EI_ASM_PREFIX(x) \
  /* BEGASM This: \
  current = PINC; // PortC Input. \
  is the same as this: */ \
  asm volatile("\t" \
  "push %0" "\t\n\t" \
  "in %0,%1" "\t\n\t" \
  : "=&r" (current) \
  : "I" (_SFR_IO_ADDR(x)) \
  ); \
  /* ENDASM ...End of the sameness.*/ \
 \
  EI_ASM_PUSHIT

// We have defined "current" as "r18" in the ISR's. */
#define EI_ASM_PREFIX_JK(x) \
  /* BEGASM This: \
  current = PINK; // PortC Input. \
  is the same as this: */ \
  asm volatile("\t" \
  "push %0" "\t\n\t" \
  "lds %0,%1" "\t\n\t" \
  : "=&r" (current) \
  : "m" (x) \
  ); \
  /* ENDASM ...End of the sameness.*/ \
 \
  EI_ASM_PUSHIT

#define EI_ASM_SUFFIX \
 asm volatile( \
  "pop r31" "\n\t" \
  "pop r30" "\n\t" \
  "pop r29" "\n\t" \
  "pop r28" "\n\t" \
  "pop r27" "\n\t" \
  "pop r26" "\n\t" \
  "pop r25" "\n\t" \
  "pop r24" "\n\t" \
  "pop r23" "\n\t" \
  "pop r22" "\n\t" \
  "pop r21" "\n\t" \
  "pop r20" "\n\t" \
  "pop r19" "\n\t" \
  "pop r0" "\n\t" \
  "out __SREG__, r0" "\t\n\t" \
  "pop r0" "\t\n\t" \
  "pop r1" "\t\n\t" \
  "pop r18" "\n\t" \
  "reti" "\t\n\t" \
  : \
  :)

/*
 * We can save a register if we hand assemble (assembler uses 4 registers for figuring out changedPins)
 * TODO!
 * ...we can also reuse registers that I think the compiler does not want to touch.
// If the compiler does not inline this, things break.
// It's not enough to have just inline, we need the attribute.
inline void __attribute__((always_inline)) inlineISRasm (uint8_t current,
  volatile uint8_t *portSnapshot,
  uint8_t risingPins, uint8_t fallingPins,
  volatile uint8_t pcmsk,
  interruptFunctionType functionPointerArray[]) {

  uint8_t i;
  uint8_t interruptMask;
  register uint8_t changedPins asm("r19");
  register uint8_t tmp1 asm("r20");
  register uint8_t tmp2 asm("r21");

  tmp1=risingPins;
  tmp1=risingPins & current;
  tmp2=fallingPins;
  tmp3=~current;
  tmp2=fallingPins & ~current;
  tmp2=tmp2 | tmp1;
  tmp3=*portSnapshot;
  tmp3=tmp3 ^ current;
  tmp3=tmp3 & tmp2;
  // changedPins=(*portSnapshot ^ current) & ((risingPins & current) | (fallingPins & ~current));

  *portSnapshot =  current;
  if (changedPins == 0) goto exitISR; // get out quickly if not interested.

  interruptMask = pcmsk & changedPins;
  if (interruptMask == 0) goto exitISR;
  i=0;
  while (1) {
    if (interruptMask & 0x01) {
      (*functionPointerArray[i])();
    }
    interruptMask=interruptMask >> 1;
    if (interruptMask == 0) goto exitISR;
    i++;
  }
  exitISR: return;
}
*/

// If the compiler does not inline this, things break.
// It's not enough to have just inline, we need the attribute.
inline void __attribute__((always_inline)) inlineISR (uint8_t current,
  volatile uint8_t *portSnapshot,
  uint8_t risingPins, uint8_t fallingPins,
  volatile uint8_t pcmsk,
  interruptFunctionType functionPointerArray[]) {

  uint8_t i;
  uint8_t interruptMask;
  uint8_t changedPins;

  //interruptSaysHello();
  changedPins=(*portSnapshot ^ current) & ((risingPins & current) | (fallingPins & ~current));
  *portSnapshot =  current;
  if (changedPins == 0) goto exitISR; // get out quickly if not interested.

  interruptMask = pcmsk & changedPins;
  if (interruptMask == 0) goto exitISR;
  i=0;
  while (1) {
    if (interruptMask & 0x01) {
      (*functionPointerArray[i])();
    }
    interruptMask=interruptMask >> 1;
    if (interruptMask == 0) goto exitISR;
    i++;
  }
  exitISR: return;
}

ISR(PORTB_VECT) {
//ISR(PORTB_VECT, ISR_NAKED) {
  uint8_t current; current=PINB;
  //register uint8_t current asm("r18");

  //EI_ASM_PREFIX(PINB);

  inlineISR(current,
      &portSnapshotB,
      risingPinsPORTB,
      fallingPinsPORTB,
      PCMSK0,
      functionPointerArrayPORTB );

  //EI_ASM_SUFFIX;
}

#if defined ARDUINO_328
ISR(PORTC_VECT, ISR_NAKED) {
  register uint8_t current asm("r18");

  EI_ASM_PREFIX(PINC);

  inlineISR(current,
      &portSnapshotC,
      risingPinsPORTC,
      fallingPinsPORTC,
      PCMSK1,
      functionPointerArrayPORTC );

  EI_ASM_SUFFIX;
}

ISR(PORTD_VECT, ISR_NAKED) {
  register uint8_t current asm("r18");

  EI_ASM_PREFIX(PIND);

  inlineISR(current,
      &portSnapshotD,
      risingPinsPORTD,
      fallingPinsPORTD,
      PCMSK2,
      functionPointerArrayPORTD );

  EI_ASM_SUFFIX;
}

#elif defined ARDUINO_MEGA
ISR(PORTJ_VECT) {
//ISR(PORTJ_VECT, ISR_NAKED) {
  uint8_t current; current=PINJ;
  //register uint8_t current asm("r18");

  //EI_ASM_PREFIX_JK(PINJ);

  inlineISR(current,
      &portSnapshotJ,
      risingPinsPORTJ,
      fallingPinsPORTJ,
      PCMSK1 >> 1, // handle PCMSK1/PORTJ weirdness...
      functionPointerArrayPORTJ );

  //EI_ASM_SUFFIX;
}

ISR(PORTK_VECT) {
//ISR(PORTK_VECT, ISR_NAKED) {
  uint8_t current; current=PINK;
  //register uint8_t current asm("r18");

  //EI_ASM_PREFIX_JK(PINK);

  inlineISR(current,
      &portSnapshotK,
      risingPinsPORTK,
      fallingPinsPORTK,
      PCMSK2,
      functionPointerArrayPORTK );

  //EI_ASM_SUFFIX;
}
#elif defined ARDUINO_LEONARDO
#error NOT IMPLEMENTED YET
#endif
