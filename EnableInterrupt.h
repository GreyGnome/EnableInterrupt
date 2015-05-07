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

#ifndef EnableInterrupt_h
#define EnableInterrupt_h
#include <Arduino.h>

// *************************************************************************************
// *************************************************************************************
// Function Prototypes *****************************************************************
// *************************************************************************************
// *************************************************************************************

// These are the only functions the end user (programmer) needs to consider. This means you!

// Arduino Due (not Duemilanove) macros. Easy-peasy.
#if defined __SAM3U4E__ || defined __SAM3X8E__ || defined __SAM3X8H__
#ifdef NEEDFORSPEED
#error The NEEDFORSPEED definition does not make sense on the Due platform.
#endif
define enableInterrupt(pin,userFunc,mode) attachInterrupt(pin, userFunc,mode)
define disableInterrupt(pin) detachInterrupt(pin)
#else
/* 
 * enableInterrupt- Sets up an interrupt on a selected Arduino pin.
 * or
 * enableInterruptFast- When used with the NEEDFORSPEED macro, sets up an interrupt on a selected Arduino pin.
 * 
 * Usage:
 * enableInterrupt(uint8_t pinNumber, void (*userFunction)(void), uint8_t mode);
 * or
 * enableInterrupt(uint8_t interruptDesignator, void (*userFunction)(void), uint8_t mode);
 *
 * For HiSpeed mode,
 * enableInterruptFast(uint8_t pinNumber, uint8_t mode);
 * or
 * enableInterruptFast(uint8_t interruptDesignator, uint8_t mode);
 *
 * ---------------------------------------------------------------------------------------
 *
 * disableInterrupt- Disables interrupt on a selected Arduino pin.
 *
 * Usage:
 *
 * disableInterrupt(uint8_t pinNumber);
 * or
 * disableInterrupt(uint8_t interruptDesignator);
 *
 * ---------------------------------------------------------------------------------------
 *
 * interruptDesignator: Essentially this is an Arduino pin, and if that's all you want to give
 * the function, it will work just fine. Why is it called an "interruptDesignator", then? Because
 * there's a twist: You can perform a bitwise "and" with the pin number and PINCHANGEINTERRUPT
 * to specify that you want to use a Pin Change Interrupt type of interrupt on those pins that
 * support both Pin Change and External Interrupts. Otherwise, the library will choose whatever
 * interrupt type (External, or Pin Change) normally applies to that pin, with priority to
 * External Interrupt. 
 *
 * Believe it or not, that complexity is all because of pins 2 and 3 on the ATmega328-based
 * Arduinos. Those are the only pins in the Arduino line that can share External or Pin Change
 * Interrupt types. Otherwise, each pin only supports a single type of interrupt and the
 * PINCHANGEINTERRUPT scheme changes nothing. This means you can ignore this whole discussion
 * for ATmega2560- or ATmega32U4-based Arduinos. You can probably safely ignore it for
 * ATmega328-based Arduinos, too.
 */
void enableInterrupt(uint8_t interruptDesignator, void (*userFunction)(void), uint8_t mode);
void disableInterrupt(uint8_t interruptDesignator);
void bogusFunctionPlaceholder(void);
#ifdef NEEDFORSPEED
#undef enableInterruptFast
// enableInterruptFast(uint8_t interruptDesignator, uint8_t mode);
#define enableInterruptFast(x, y) enableInterrupt(x, bogusFunctionPlaceholder, y)
#endif


// *************************************************************************************
// End Function Prototypes *************************************************************
// *************************************************************************************

#undef PINCHANGEINTERRUPT
#define PINCHANGEINTERRUPT 0x80

#undef attachPinChangeInterrupt
#undef detachPinChangeInterrupt
#define detachPinChangeInterrupt(pin)                   disableInterrupt(pin)
#define attachPinChangeInterrupt(pin,userFunc,mode)     enableInterrupt(pin , userFunc,mode)

#ifndef LIBCALL_ENABLEINTERRUPT // LIBCALL_ENABLEINTERRUPT ****************************************
#ifdef NEEDFORSPEED
void bogusFunctionPlaceholder(void) {
}
#include "pindefs_speed.h"
#endif

// Example: EI_printPSTR("This is a nice long string that takes no static ram");
#define EI_printPSTR(x) SerialPrint_P(PSTR(x))
void SerialPrint_P(const char *str) {
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

#ifndef NEEDFORSPEED
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
// 2 of the interrupts are unsupported on Arduino UNO.
struct functionPointersPortB {
  interruptFunctionType pinZero;
  interruptFunctionType pinOne;
  interruptFunctionType pinTwo;
  interruptFunctionType pinThree;
  interruptFunctionType pinFour;
  interruptFunctionType pinFive;
};
typedef struct functionPointersPortB functionPointersPortB;

functionPointersPortB portBFunctions = { NULL, NULL, NULL, NULL, NULL, NULL };

// 1 of the interrupts are used as RESET on Arduino UNO.
struct functionPointersPortC {
  interruptFunctionType pinZero;
  interruptFunctionType pinOne;
  interruptFunctionType pinTwo;
  interruptFunctionType pinThree;
  interruptFunctionType pinFour;
  interruptFunctionType pinFive;
};
typedef struct functionPointersPortC functionPointersPortC;

functionPointersPortC portCFunctions = { NULL, NULL, NULL, NULL, NULL, NULL };

// 1 of the interrupts are used as RESET on Arduino UNO.
struct functionPointersPortD {
  interruptFunctionType pinZero;
  interruptFunctionType pinOne;
  interruptFunctionType pinTwo;
  interruptFunctionType pinThree;
  interruptFunctionType pinFour;
  interruptFunctionType pinFive;
  interruptFunctionType pinSix;
  interruptFunctionType pinSeven;
};
typedef struct functionPointersPortD functionPointersPortD;

functionPointersPortD portDFunctions = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };
#endif // NEEDFORSPEED

// For Pin Change Interrupts; since we're duplicating FALLING and RISING in software,
// we have to know how the ports were defined.
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

volatile uint8_t portJPCMSK=0; // This is a shifted version of PCMSK for PortJ, so I
			                         //	don't have to perform a shift in the IRQ.

#ifndef NEEDFORSPEED
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

struct functionPointersPortB {
  interruptFunctionType pinZero;
  interruptFunctionType pinOne;
  interruptFunctionType pinTwo;
  interruptFunctionType pinThree;
  interruptFunctionType pinFour;
  interruptFunctionType pinFive;
  interruptFunctionType pinSix;
  interruptFunctionType pinSeven;
};
typedef struct functionPointersPortB functionPointersPortB;

functionPointersPortB portBFunctions = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };

// only 7 pins total of port J are supported as interrupts on the ATmega2560,
// and only PJ0 and 1 are supported on the Arduino MEGA.
// For PCI1 the 0th bit is PE0.   PJ2-6 are not exposed on the Arduino pins, but
// we will support them anyway. There are clones that provide them, and users may
// solder in their own connections (...go, Makers!)
struct functionPointersPortJ {
  interruptFunctionType pinZero;
  interruptFunctionType pinOne;
  interruptFunctionType pinTwo;
  interruptFunctionType pinThree;
  interruptFunctionType pinFour;
  interruptFunctionType pinFive;
  interruptFunctionType pinSix;
};
typedef struct functionPointersPortJ functionPointersPortJ;

functionPointersPortJ portJFunctions = { NULL, NULL, NULL, NULL, NULL, NULL, NULL };

struct functionPointersPortK {
  interruptFunctionType pinZero;
  interruptFunctionType pinOne;
  interruptFunctionType pinTwo;
  interruptFunctionType pinThree;
  interruptFunctionType pinFour;
  interruptFunctionType pinFive;
  interruptFunctionType pinSix;
  interruptFunctionType pinSeven;
};
typedef struct functionPointersPortK functionPointersPortK;

functionPointersPortK portKFunctions = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };
#endif // NEEDFORSPEED

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
#elif defined __AVR_ATmega32U4__ || defined __AVR_ATmega16U4__
#define ARDUINO_LEONARDO

/* To derive this list: 
   sed -n -e '1,/digital_pin_to_port_PGM/d' -e '/^}/,$d' -e '/P/p' \
       /usr/share/arduino/hardware/arduino/variants/leonardo/pins_arduino.h | \
       awk '{print "  ", $5 ", // " $5 "  pin: " $3}'
   ...then massage the output as necessary to create the below:
*/

#ifndef NEEDFORSPEED
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
  6, // PB6  pin: D10 // would make the code any shorter.
  7, // PB7  pin: D11
  6, // PD6  pin: D12
  7, // PC7  pin: D13
  3, // PB3  pin: D14 MISO
  1, // PB1  pin: D15 SCK
  2, // PB2  pin: D16 MOSI
  0, // PB0  pin: D17 SS (RXLED). Available on non-Leonardo 32u4 boards, at least (exposed on the Leonardo??)
// There are no ports we care about after pin 17.
};

interruptFunctionType functionPointerArrayEXTERNAL[5];
struct functionPointersPortB {
  interruptFunctionType pinZero;
  interruptFunctionType pinOne;
  interruptFunctionType pinTwo;
  interruptFunctionType pinThree;
  interruptFunctionType pinFour;
  interruptFunctionType pinFive;
  interruptFunctionType pinSix;
  interruptFunctionType pinSeven;
};
typedef struct functionPointersPortB functionPointersPortB;

functionPointersPortB portBFunctions = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };

// For Pin Change Interrupts; since we're duplicating FALLING and RISING in software,
// we have to know how we were defined.
volatile uint8_t risingPinsPORTB=0;
volatile uint8_t fallingPinsPORTB=0;

// for the saved state of the ports
static volatile uint8_t portSnapshotB;
#endif // NEEDFOR SPEED

#define PORTB_VECT PCINT0_vect
#endif // #if defined __AVR_ATmega168__ || defined __AVR_ATmega168A__ ...

// ===========================================================================================
// END END END DATA STRUCTURES ===============================================================
// ===========================================================================================

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
#ifndef NEEDFORSPEED
  uint8_t portBitNumber; // when an interrupted pin is found, this will be used to choose the function.
  interruptFunctionType *calculatedPointer;
#endif

  arduinoPin=interruptDesignator & ~PINCHANGEINTERRUPT;

  // *************************************************************************************
  // *************************************************************************************
  // Pin Change Interrupts
  // *************************************************************************************
  // *************************************************************************************
#if defined ARDUINO_328
  if ( (interruptDesignator & PINCHANGEINTERRUPT) || (arduinoPin != 2 && arduinoPin != 3) ) {
#elif defined ARDUINO_MEGA
  // NOTE: PJ2-6 and PE6 & 7 are not exposed on the Arduino, but they are supported here
  // for software interrupts and support of non-Arduino platforms which expose more pins.
  // PJ2-6 are called pins 70-74, PE6 is pin 75, PE7 is pin 76.
  if ( (arduinoPin != 2 && arduinoPin != 3 && arduinoPin != 75 && arduinoPin != 76
                                           && (arduinoPin < 18 || arduinoPin > 21))
     ) {
    if (arduinoPin > 69) { // Dastardly tricks to support PortJ 2-7
      portMask=pgm_read_byte(&digital_pin_to_bit_mask_PGM[arduinoPin-6]); // Steal from PK
      portNumber=PJ;
    } else
#elif defined ARDUINO_LEONARDO
  if ( (arduinoPin > 3) && (arduinoPin != 7) ) {
#else
#error Unsupported Arduino platform
#endif
    {
      portMask=pgm_read_byte(&digital_pin_to_bit_mask_PGM[arduinoPin]);
      portNumber=pgm_read_byte(&digital_pin_to_port_PGM[arduinoPin]);
    }

    // save the mode
    if ((mode == RISING) || (mode == CHANGE)) {
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
      }
      if (portNumber==PK) {
        risingPinsPORTK |= portMask;
      }
#elif defined ARDUINO_LEONARDO
      // No other Pin Change Interrupt ports than B on Leonardo
#endif
    }
    if ((mode == FALLING) || (mode == CHANGE)) {
      if (portNumber==PB) {
        fallingPinsPORTB |= portMask;
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
      }
      if (portNumber==PK) {
        fallingPinsPORTK |= portMask;
      }
#elif defined ARDUINO_LEONARDO
      // No other Pin Change Interrupt ports than B on Leonardo
#endif
    }

#ifndef NEEDFORSPEED
    // assign the function to be run in the ISR
    // save the initial value of the port
    portBitNumber=pgm_read_byte(&digital_pin_to_port_bit_number_PGM[arduinoPin]);
#endif
#if defined ARDUINO_328
    if (portNumber==PC) {
#ifndef NEEDFORSPEED
      calculatedPointer=&portCFunctions.pinZero + portBitNumber;
      *calculatedPointer = userFunction;
#endif

      portSnapshotC=*portInputRegister(portNumber);
      pcmsk=&PCMSK1;
      PCICR |= _BV(1);
    }
    if (portNumber==PD) {
#ifndef NEEDFORSPEED
      calculatedPointer=&portDFunctions.pinZero + portBitNumber;
      *calculatedPointer = userFunction;
#endif

      portSnapshotD=*portInputRegister(portNumber);
      pcmsk=&PCMSK2;
      PCICR |= _BV(2);
    }
#elif defined ARDUINO_MEGA
    if (portNumber==PJ) {
#ifndef NEEDFORSPEED
      calculatedPointer=&portJFunctions.pinZero + portBitNumber;
      *calculatedPointer = userFunction;
#endif

      portSnapshotJ=*portInputRegister(portNumber);
      pcmsk=&PCMSK1;
      PCICR |= _BV(1);
      portJPCMSK|=portMask; // because PCMSK1 is shifted wrt. PortJ.
      portMask <<= 1; // Handle port J's oddness. PJ0 is actually 1 on PCMSK1.
    }
    if (portNumber==PK) {
#ifndef NEEDFORSPEED
      calculatedPointer=&portKFunctions.pinZero + portBitNumber;
      *calculatedPointer = userFunction;
#endif

      portSnapshotK=*portInputRegister(portNumber);
      pcmsk=&PCMSK2;
      PCICR |= _BV(2);
    }
#elif defined ARDUINO_LEONARDO
      // No other Pin Change Interrupt ports than B on Leonardo
#endif // defined ARDUINO_328
    if (portNumber==PB) {
#ifndef NEEDFORSPEED
      calculatedPointer=&portBFunctions.pinZero + portBitNumber;
      *calculatedPointer = userFunction;
#endif

      portSnapshotB=*portInputRegister(portNumber);
      pcmsk=&PCMSK0;
      PCICR |= _BV(0);
    }
    *pcmsk |= portMask;  // appropriate bit, e.g. this could be PCMSK1 |= portMask;

    // With the exception of the Global Interrupt Enable bit in SREG, interrupts on the arduinoPin
    // are now ready. GIE may have already been set on a previous enable, so it's important
    // to take note of the order in which things were done, above.

  // *************************************************************************************
  // *************************************************************************************
  // External Interrupts
  // *************************************************************************************
  // *************************************************************************************
  } else {
    uint8_t origSREG; // to save for interrupts
    origSREG = SREG;
    cli(); // no interrupts while we're setting up an interrupt.
#if defined ARDUINO_328
    if (arduinoPin == 3) {
#ifndef NEEDFORSPEED
      functionPointerArrayEXTERNAL[1] = userFunction;
#endif
      EIMSK &= ~_BV(1);
      EICRA &= (~_BV(2) & ~_BV(3));
      EICRA |= mode << 2;
      EIFR  |= _BV(1); // using a clue from the ATmega2560 datasheet.
      EIMSK |= _BV(1);
    } else {
#ifndef NEEDFORSPEED
      functionPointerArrayEXTERNAL[0] = userFunction;
#endif
      EIMSK &= ~_BV(0);
      EICRA &= (~_BV(0) & ~_BV(1));
      EICRA |= mode;
      EIFR  |= _BV(0); // using a clue from the ATmega2560 datasheet.
      EIMSK |= _BV(0);
    }
#elif defined ARDUINO_MEGA
    switch (arduinoPin) {
      case 21 : // INT0
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[0] = userFunction;
#endif
        EIMSK &= ~_BV(0);
        EICRA &= (~_BV(0) & ~_BV(1));
        EICRA |= mode;
        EIFR |= _BV(0);
        EIMSK |= _BV(0);
        break;
      case 20 : // INT1
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[1] = userFunction;
#endif
        EIMSK &= ~_BV(1);
        EICRA &= (~_BV(2) & ~_BV(3));
        EICRA |= (mode << 2);
        EIFR |= _BV(1);
        EIMSK |= _BV(1);
        break;
      case 19 : // INT2
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[2] = userFunction;
#endif
        EIMSK &= ~_BV(2);
        EICRA &= (~_BV(4) & ~_BV(5));
        EICRA |= (mode << 4);
        EIFR |= _BV(2);
        EIMSK |= _BV(2);
        break;
      case 18 : // INT3
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[3] = userFunction;
#endif
        EIMSK &= ~_BV(3);
        EICRA &= (~_BV(6) & ~_BV(7));
        EICRA |= (mode << 6);
        EIFR |= _BV(3);
        EIMSK |= _BV(3);
        break;
      case  2 : // INT4
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[4] = userFunction;
#endif
        EIMSK &= ~_BV(4);
        EICRB &= (~_BV(0) & ~_BV(1));
        EICRB |= mode;
        EIFR |= _BV(4);
        EIMSK |= _BV(4);
        break;
      case  3 : // INT5
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[5] = userFunction;
#endif
        EIMSK &= ~_BV(5);
        EICRB &= (~_BV(2) & ~_BV(3));
        EICRB |= (mode << 2);
        EIFR |= _BV(5);
        EIMSK |= _BV(5);
        break;
      case 75 : // INT6- Fake Arduino Pin
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[6] = userFunction;
#endif
        EIMSK &= ~_BV(6);
        EICRB &= (~_BV(4) & ~_BV(5));
        EICRB |= (mode << 4);
        EIFR |= _BV(6);
        EIMSK |= _BV(6);
        break;
      case 76 : // INT7- Fake Arduino Pin
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[7] = userFunction;
#endif
        EIMSK &= ~_BV(7);
        EICRB &= (~_BV(6) & ~_BV(7));
        EICRB |= (mode << 6);
        EIFR |= _BV(7);
        EIMSK |= _BV(7);
        break;
    }
#elif defined ARDUINO_LEONARDO
    switch (arduinoPin) {
      case 3 : // INT0
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[0] = userFunction;
#endif
        EIMSK &= ~_BV(0);
        EICRA &= (~_BV(0) & ~_BV(1));
        EICRA |= mode;
        EIFR |= _BV(0);
        EIMSK |= _BV(0);
        break;
      case 2 : // INT1
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[1] = userFunction;
#endif
        EIMSK &= ~_BV(1);
        EICRA &= (~_BV(2) & ~_BV(3));
        EICRA |= (mode << 2);
        EIFR |= _BV(1);
        EIMSK |= _BV(1);
        break;
      case 0 : // INT2
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[2] = userFunction;
#endif
        EIMSK &= ~_BV(2);
        EICRA &= (~_BV(4) & ~_BV(5));
        EICRA |= (mode << 4);
        EIFR |= _BV(2);
        EIMSK |= _BV(2);
        break;
      case 1 : // INT3
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[3] = userFunction;
#endif
        EIMSK &= ~_BV(3);
        EICRA &= (~_BV(6) & ~_BV(7));
        EICRA |= (mode << 6);
        EIFR |= _BV(3);
        EIMSK |= _BV(3);
        break;
      case 7 : // INT6
#ifndef NEEDFORSPEED
        functionPointerArrayEXTERNAL[4] = userFunction;
#endif
        EIMSK &= ~_BV(6);
        EICRB &= (~_BV(4) & ~_BV(5));
        EICRB |= (mode << 4);
        EIFR |= _BV(6);
        EIMSK |= _BV(6);
        break;
    }
#endif
    SREG=origSREG;
  }
  SREG |= (1 << SREG_I); // GIE bit in SREG. From /usr/avr/include/avr/common.h
}

void disableInterrupt (uint8_t interruptDesignator) {

  uint8_t origSREG; // to save for interrupts
  uint8_t arduinoPin;
  uint8_t portNumber=0;
  uint8_t portMask=0;

  origSREG = SREG;
  cli();
  arduinoPin=interruptDesignator & ~PINCHANGEINTERRUPT;
#if defined ARDUINO_328
  if ( (interruptDesignator & PINCHANGEINTERRUPT) || (arduinoPin != 2 && arduinoPin != 3) ) {
#elif defined ARDUINO_MEGA
  // NOTE: PJ2-6 and PE6 & 7 are not exposed on the Arduino, but they are supported here
  // for software interrupts and support of non-Arduino platforms which expose more pins.
  // PJ2-6 are called pins 70-74, PE6 is pin 75, PE7 is pin 76.
  if ( (arduinoPin != 2 && arduinoPin != 3 && arduinoPin != 75 && arduinoPin != 76
                                           && (arduinoPin < 18 || arduinoPin > 21))
     ) {
    if (arduinoPin > 69) { // Dastardly tricks to support PortJ 2-7
      portMask=pgm_read_byte(&digital_pin_to_bit_mask_PGM[arduinoPin-6]); // Steal from PK
      portNumber=PJ;
    } else
#elif defined ARDUINO_LEONARDO
  if ( (arduinoPin > 3) && (arduinoPin != 7) ) {
#else
#error Unsupported Arduino platform
#endif
    {
      portMask=pgm_read_byte(&digital_pin_to_bit_mask_PGM[arduinoPin]);
      portNumber=pgm_read_byte(&digital_pin_to_port_PGM[arduinoPin]);
    }
    if (portNumber == PB) {
      PCMSK0 &= ~portMask;
      if (PCMSK0 == 0) { PCICR &= ~_BV(0); };
      risingPinsPORTB &= ~portMask;
      fallingPinsPORTB &= ~portMask;
    }
#if defined ARDUINO_328
    if (portNumber == PC) {
      PCMSK1 &= ~portMask;
      if (PCMSK1 == 0) { PCICR &= ~_BV(1); };
      risingPinsPORTC &= ~portMask;
      fallingPinsPORTC &= ~portMask;
    }
    if (portNumber == PD) {
      PCMSK2 &= ~portMask;
      if (PCMSK2 == 0) { PCICR &= ~_BV(2); };
      risingPinsPORTD &= ~portMask;
      fallingPinsPORTD &= ~portMask;
    }
#elif defined ARDUINO_MEGA
    if (portNumber == PJ) {
      // Handle port J's oddness. PJ0 is actually 1 on PCMSK1.
      PCMSK1 &= ((~portMask << 1) | 0x01); // or with 1 to not touch PE0.
      if (PCMSK1 == 0) { PCICR &= ~_BV(1); };
      risingPinsPORTJ &= ~portMask;
      fallingPinsPORTJ &= ~portMask;
    }
    if (portNumber == PK) {
      PCMSK2 &= ~portMask;
      if (PCMSK2 == 0) { PCICR &= ~_BV(2); };
      risingPinsPORTK &= ~portMask;
      fallingPinsPORTK &= ~portMask;
    }
#elif defined ARDUINO_LEONARDO
    // No other Pin Change Interrupt ports than B on Leonardo
#endif
  } else {
#if defined ARDUINO_328
    if (arduinoPin == 3) {
      EIMSK &= ~_BV(1);
      EICRA &= (~_BV(2) & ~_BV(3));
      EIFR  |= _BV(1); // using a clue from the ATmega2560 datasheet.
    } else {
      EIMSK &= ~_BV(0);
      EICRA &= (~_BV(0) & ~_BV(1));
      EIFR  |= _BV(0); // using a clue from the ATmega2560 datasheet.
    }
#elif defined ARDUINO_MEGA
    switch (arduinoPin) {
      case 21 : // INT0
        EIMSK &= ~_BV(0);
        EICRA &= (~_BV(0) & ~_BV(1));
        EIFR |= _BV(0);
        break;
      case 20 : // INT1
        EIMSK &= ~_BV(1);
        EICRA &= (~_BV(2) & ~_BV(3));
        EIFR |= _BV(1);
        break;
      case 19 : // INT2
        EIMSK &= ~_BV(2);
        EICRA &= (~_BV(4) & ~_BV(5));
        EIFR |= _BV(2);
        break;
      case 18 : // INT3
        EIMSK &= ~_BV(3);
        EICRA &= (~_BV(6) & ~_BV(7));
        EIFR |= _BV(3);
        break;
      case  2 : // INT4
        EIMSK &= ~_BV(4);
        EICRB &= (~_BV(0) & ~_BV(1));
        EIFR |= _BV(4);
        break;
      case  3 : // INT5
        EIMSK &= ~_BV(5);
        EICRB &= (~_BV(2) & ~_BV(3));
        EIFR |= _BV(5);
        break;
      case 75 : // INT6- Fake Arduino Pin
        EIMSK &= ~_BV(6);
        EICRB &= (~_BV(4) & ~_BV(5));
        EIFR |= _BV(6);
        break;
      case 76 : // INT7- Fake Arduino Pin
        EIMSK &= ~_BV(7);
        EICRB &= (~_BV(6) & ~_BV(7));
        EIFR |= _BV(7);
        break;
    }
#elif defined ARDUINO_LEONARDO
    switch (arduinoPin) {
      case 3 : // INT0
        EIMSK &= ~_BV(0);
        EICRA &= (~_BV(0) & ~_BV(1));
        EIFR |= _BV(0);
        break;
      case 2 : // INT1
        EIMSK &= ~_BV(1);
        EICRA &= (~_BV(2) & ~_BV(3));
        EIFR |= _BV(1);
        break;
      case 0 : // INT2
        EIMSK &= ~_BV(2);
        EICRA &= (~_BV(4) & ~_BV(5));
        EIFR |= _BV(2);
        break;
      case 1 : // INT3
        EIMSK &= ~_BV(3);
        EICRA &= (~_BV(6) & ~_BV(7));
        EIFR |= _BV(3);
        break;
      case 7 : // INT6
        EIMSK &= ~_BV(6);
        EICRB &= (~_BV(4) & ~_BV(5));
        EIFR |= _BV(6);
        break;
    }
#endif
  }
  SREG = origSREG;
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
////////////////////// ISRs /////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
ISR(INT0_vect) {
#ifndef NEEDFORSPEED
  (*functionPointerArrayEXTERNAL[0])();
#else
#if defined ARDUINO_MEGA
#ifdef INTERRUPT_FLAG_PIN21
  INTERRUPT_FLAG_PIN21++;
#endif
#endif
#if defined ARDUINO_LEONARDO
#ifdef INTERRUPT_FLAG_PIN3
  INTERRUPT_FLAG_PIN3++;
#endif
#endif
#if defined ARDUINO_328
#ifdef INTERRUPT_FLAG_PIN2
  INTERRUPT_FLAG_PIN2++;
#endif
#endif
#endif // NEEDFORSPEED
}

ISR(INT1_vect) {
#ifndef NEEDFORSPEED
  (*functionPointerArrayEXTERNAL[1])();
#else
#if defined ARDUINO_MEGA
#ifdef INTERRUPT_FLAG_PIN20
  INTERRUPT_FLAG_PIN20++;
#endif
#endif
#if defined ARDUINO_LEONARDO
#ifdef INTERRUPT_FLAG_PIN2
  INTERRUPT_FLAG_PIN2++;
#endif
#endif
#if defined ARDUINO_328
#ifdef INTERRUPT_FLAG_PIN3
  INTERRUPT_FLAG_PIN3++;
#endif
#endif
#endif // NEEDFORSPEED
}

#if defined ARDUINO_MEGA || defined ARDUINO_LEONARDO
ISR(INT2_vect) {
#ifndef NEEDFORSPEED
  (*functionPointerArrayEXTERNAL[2])();
#else
#if defined ARDUINO_MEGA
#ifdef INTERRUPT_FLAG_PIN19
  INTERRUPT_FLAG_PIN19++;
#endif
#else
#ifdef INTERRUPT_FLAG_PIN0
  INTERRUPT_FLAG_PIN0++;
#endif
#endif
#endif // NEEDFORSPEED
}

ISR(INT3_vect) {
#ifndef NEEDFORSPEED
  (*functionPointerArrayEXTERNAL[3])();
#else
#if defined ARDUINO_MEGA
#ifdef INTERRUPT_FLAG_PIN18
  INTERRUPT_FLAG_PIN18++;
#endif
#else
#ifdef INTERRUPT_FLAG_PIN1
  INTERRUPT_FLAG_PIN1++;
#endif
#endif
#endif // NEEDFORSPEED
}
#endif // ARDUINO_MEGA || ARDUINO_LEONARDO

#if defined ARDUINO_MEGA
ISR(INT4_vect) {
#ifndef NEEDFORSPEED
  (*functionPointerArrayEXTERNAL[4])();
#else
#ifdef INTERRUPT_FLAG_PIN2
  INTERRUPT_FLAG_PIN2++;
#endif
#endif // NEEDFORSPEED
}

ISR(INT5_vect) {
#ifndef NEEDFORSPEED
  (*functionPointerArrayEXTERNAL[5])();
#else
#ifdef INTERRUPT_FLAG_PIN3
  INTERRUPT_FLAG_PIN3++;
#endif
#endif // NEEDFORSPEED
}

ISR(INT6_vect) {
#ifndef NEEDFORSPEED
  (*functionPointerArrayEXTERNAL[6])();
#else
#ifdef INTERRUPT_FLAG_PIN75
  INTERRUPT_FLAG_PIN75++;
#endif
#endif // NEEDFORSPEED
}

ISR(INT7_vect) {
#ifndef NEEDFORSPEED
  (*functionPointerArrayEXTERNAL[7])();
#else
#ifdef INTERRUPT_FLAG_PIN76
  INTERRUPT_FLAG_PIN76++;
#endif
#endif // NEEDFORSPEED
}
#endif // defined ARDUINO_MEGA

#if defined ARDUINO_LEONARDO
ISR(INT6_vect) {
#ifndef NEEDFORSPEED
  (*functionPointerArrayEXTERNAL[4])();
#else
#ifdef INTERRUPT_FLAG_PIN7
  INTERRUPT_FLAG_PIN7++;
#endif
#endif // NEEDFORSPEED
}
#endif // defined ARDUINO_LEONARDO

ISR(PORTB_VECT) {
  uint8_t current;
  uint8_t interruptMask;
  uint8_t changedPins;
  uint8_t tmp;

  current=PINB;
//  changedPins=(portSnapshotB ^ current) &
//                                       ((risingPinsPORTB & current) | (fallingPinsPORTB & ~current));
  changedPins   = portSnapshotB ^ current;
  tmp           = risingPinsPORTB & current;
  interruptMask = fallingPinsPORTB & ~current; // steal interruptMask as a temp variable
  interruptMask = interruptMask | tmp;
  interruptMask = changedPins & interruptMask;
  interruptMask = PCMSK0 & interruptMask;


  portSnapshotB = current;
#ifdef NEEDFORSPEED
#include "portb_speed.h"
#else
  if (interruptMask == 0) goto exitPORTBISR; // get out quickly if not interested.
  if (interruptMask & _BV(0)) portBFunctions.pinZero();
  if (interruptMask & _BV(1)) portBFunctions.pinOne();
  if (interruptMask & _BV(2)) portBFunctions.pinTwo();
  if (interruptMask & _BV(3)) portBFunctions.pinThree();
  if (interruptMask & _BV(4)) portBFunctions.pinFour();
  if (interruptMask & _BV(5)) portBFunctions.pinFive();
#ifndef ARDUINO_328
  if (interruptMask & _BV(6)) portBFunctions.pinSix();
  if (interruptMask & _BV(7)) portBFunctions.pinSeven();
#endif
  exitPORTBISR: return;
  // FOR MEASUREMENT ONLY
  // exitPORTBISR: PORTC &= ~(1 << PC5); // SIGNAL THAT WE ARE LEAVING THE INTERRUPT
#endif // NEEDFORSPEED
}

#if defined ARDUINO_328
ISR(PORTC_VECT) {
  uint8_t current;
  uint8_t interruptMask;
  uint8_t changedPins;
  uint8_t tmp;

  current=PINC;
//  changedPins=(portSnapshotB ^ current) &
//                                       ((risingPinsPORTB & current) | (fallingPinsPORTB & ~current));
  changedPins   = portSnapshotC ^ current;
  tmp           = risingPinsPORTC & current;
  interruptMask = fallingPinsPORTC & ~current; // steal interruptMask as a temp variable
  interruptMask = interruptMask | tmp;
  interruptMask = changedPins & interruptMask;
  interruptMask = PCMSK1 & interruptMask;

  portSnapshotC = current;
#ifdef NEEDFORSPEED
#include "portc_speed.h"
#else
  if (interruptMask == 0) goto exitPORTCISR; // get out quickly if not interested.
  if (interruptMask & _BV(0)) portCFunctions.pinZero();
  if (interruptMask & _BV(1)) portCFunctions.pinOne();
  if (interruptMask & _BV(2)) portCFunctions.pinTwo();
  if (interruptMask & _BV(3)) portCFunctions.pinThree();
  if (interruptMask & _BV(4)) portCFunctions.pinFour();
  if (interruptMask & _BV(5)) portCFunctions.pinFive();
  exitPORTCISR: return;
#endif // NEEDFORSPEED
}

ISR(PORTD_VECT) {
  uint8_t current;
  uint8_t interruptMask;
  uint8_t changedPins;
  uint8_t tmp;

  current=PIND;
//  changedPins=(portSnapshotB ^ current) &
//                                       ((risingPinsPORTB & current) | (fallingPinsPORTB & ~current));
  changedPins   = portSnapshotD ^ current;
  tmp           = risingPinsPORTD & current;
  interruptMask = fallingPinsPORTD & ~current; // steal interruptMask as a temp variable
  interruptMask = interruptMask | tmp;
  interruptMask = changedPins & interruptMask;
  interruptMask = PCMSK2 & interruptMask;


  portSnapshotD = current;
#ifdef NEEDFORSPEED
#include "portd_speed.h"
#else
  if (interruptMask == 0) goto exitPORTDISR; // get out quickly if not interested.
  if (interruptMask & _BV(0)) portDFunctions.pinZero();
  if (interruptMask & _BV(1)) portDFunctions.pinOne();
  if (interruptMask & _BV(2)) portDFunctions.pinTwo();
  if (interruptMask & _BV(3)) portDFunctions.pinThree();
  if (interruptMask & _BV(4)) portDFunctions.pinFour();
  if (interruptMask & _BV(5)) portDFunctions.pinFive();
  if (interruptMask & _BV(6)) portDFunctions.pinSix();
  if (interruptMask & _BV(7)) portDFunctions.pinSeven();
  exitPORTDISR: return;
#endif // NEEDFORSPEED
}

#elif defined ARDUINO_MEGA
ISR(PORTJ_VECT) {
  uint8_t current;
  uint8_t interruptMask;
  uint8_t changedPins;
  uint8_t tmp;

  current=PINJ;
//  changedPins=(portSnapshotB ^ current) &
//                                       ((risingPinsPORTB & current) | (fallingPinsPORTB & ~current));
  changedPins   = portSnapshotJ ^ current;
  tmp           = risingPinsPORTJ & current;
  interruptMask = fallingPinsPORTJ & ~current; // steal interruptMask as a temp variable
  interruptMask = interruptMask | tmp;
  interruptMask = changedPins & interruptMask;
  interruptMask = portJPCMSK & interruptMask; // because PCMSK1 is shifted wrt. PortJ.

  portSnapshotJ = current;
#ifdef NEEDFORSPEED
#include "portj_speed.h"
#else
  if (interruptMask == 0) goto exitPORTJISR; // get out quickly if not interested.
  if (interruptMask & _BV(0)) portJFunctions.pinZero();
  if (interruptMask & _BV(1)) portJFunctions.pinOne();
  if (interruptMask & _BV(2)) portJFunctions.pinTwo();
  if (interruptMask & _BV(3)) portJFunctions.pinThree();
  if (interruptMask & _BV(4)) portJFunctions.pinFour();
  if (interruptMask & _BV(5)) portJFunctions.pinFive();
  if (interruptMask & _BV(6)) portJFunctions.pinSix();
  exitPORTJISR: return;
#endif // NEEDFORSPEED
}

ISR(PORTK_VECT) {
  uint8_t current;
  uint8_t interruptMask;
  uint8_t changedPins;
  uint8_t tmp;

  current=PINK;
//  changedPins=(portSnapshotB ^ current) &
//                                       ((risingPinsPORTB & current) | (fallingPinsPORTB & ~current));
  changedPins   = portSnapshotK ^ current;
  tmp           = risingPinsPORTK & current;
  interruptMask = fallingPinsPORTK & ~current; // steal interruptMask as a temp variable
  interruptMask = interruptMask | tmp;
  interruptMask = changedPins & interruptMask;
  interruptMask = PCMSK2 & interruptMask;

  portSnapshotK = current;
#ifdef NEEDFORSPEED
#include "portk_speed.h"
#else
  if (interruptMask == 0) goto exitPORTKISR; // get out quickly if not interested.
  if (interruptMask & _BV(0)) portKFunctions.pinZero();
  if (interruptMask & _BV(1)) portKFunctions.pinOne();
  if (interruptMask & _BV(2)) portKFunctions.pinTwo();
  if (interruptMask & _BV(3)) portKFunctions.pinThree();
  if (interruptMask & _BV(4)) portKFunctions.pinFour();
  if (interruptMask & _BV(5)) portKFunctions.pinFive();
  if (interruptMask & _BV(6)) portKFunctions.pinSix();
  if (interruptMask & _BV(7)) portKFunctions.pinSeven();
  exitPORTKISR: return;
#endif // NEEDFORSPEED
}
#elif defined ARDUINO_LEONARDO
  // No other Pin Change Interrupt ports than B on Leonardo
#endif // defined ARDUINO_328

#endif // #ifndef LIBCALL_ENABLEINTERRUPT *********************************************************
#endif // #if defined __SAM3U4E__ || defined __SAM3X8E__ || defined __SAM3X8H__
#endif // #ifndef EnableInterrupt_h ***************************************************************
