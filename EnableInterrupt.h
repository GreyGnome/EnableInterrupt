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
/* These are the only functions then end user (programmer) needs to consider. This means you! */

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

inline void interruptSaysHello() {
  uint8_t led_on, led_off;         // DEBUG
  led_on=0b00100000; led_off=0b0;  // PB5 == Arduino pin 13. Note that this will overwrite the rest of PortB.

  PORTB=led_off;
  PORTB=led_on;
  PORTB=led_off;
  PORTB=led_on;
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
};

interruptFunctionType functionPointerArrayEXTERNAL[6];
interruptFunctionType functionPointerArrayPORTB[8];
// only 7 pins total of port J are supported as interrupts on the ATmega2560,
// and only PJ1 and 2 are supported on the Arduino MEGA.
// For PCI1 the 0th bit is PE0.   PJ2-6 are not exposed on the Arduino pins, but
// we will support them anyway. There are clones that provide them, and users may
// solder in their own connections (go, Makers!)
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
  uint8_t portBitNumber; // when an interrupted pin is found, this will be used to choose the function.

  arduinoPin=interruptDesignator & ~PINCHANGEINTERRUPT;
  printPSTR("Arduino pin is "); Serial.println(arduinoPin, DEC); // OK-MIKE

#if defined ARDUINO_328
	if ( (interruptDesignator && PINCHANGEINTERRUPT) || (arduinoPin != 2 && arduinoPin != 3) ) {
#elif defined ARDUINO_MEGA
	if ( (interruptDesignator && PINCHANGEINTERRUPT) || (arduinoPin != 2 && arduinoPin != 3 &&
                                                       (arduinoPin < 18 || arduinoPin > 21) )
      ) {
#else
#error Unsupported Arduino platform
#endif
    portMask=pgm_read_byte(&digital_pin_to_bit_mask_PGM[arduinoPin]);
    printPSTR("portMask is 0x"); Serial.println(portMask, HEX);

    portNumber=pgm_read_byte(&digital_pin_to_port_PGM[arduinoPin]);
    if (portNumber == PJ) { portMask << 1; }; // Handle port J's oddness. PJ0 is actually 1 on PCMSK1.

    printPSTR("portNumber is "); Serial.println(portNumber, HEX);  // OK-MIKE

    // save the mode
    if ((mode == RISING) || (mode == CHANGE)) {
      printPSTR("Mode is change\r\n");
      if (portNumber==PB) {
        risingPinsPORTB |= portMask;
      }
      if (portNumber==PC) {
        risingPinsPORTC |= portMask;
        printPSTR("Port C, rising pins 0x"); // OK-MIKE
        Serial.println(risingPinsPORTC, HEX);
        printPSTR("Inital value of port: 0x");
        Serial.println(*portInputRegister(portNumber), HEX);
      }
      if (portNumber==PD) {
        risingPinsPORTD |= portMask;
      }
    }
    if ((mode == FALLING) || (mode == CHANGE)) {
      printPSTR("Mode is change\r\n");
      if (portNumber==PB) {
        fallingPinsPORTB |= portMask;
      }
      if (portNumber==PC) {
        fallingPinsPORTC |= portMask;
        printPSTR("Port C, falling pins 0x"); // OK-MIKE
        Serial.println(fallingPinsPORTC, HEX);
        printPSTR("PCMSK1 is 0x"); Serial.println(PCMSK1, HEX);
      }
      if (portNumber==PD) {
        fallingPinsPORTD |= portMask;
      }
    }

    // set the appropriate PCICR bit
    printPSTR("PCICR bit: 0x");
    Serial.println(digitalPinToPCICRbit(arduinoPin), HEX);
    PCICR |= (1 << digitalPinToPCICRbit(arduinoPin)); // OK-MIKE
    // assign the function to be run in the ISR
    // save the initial value of the port
    portBitNumber=pgm_read_byte(&digital_pin_to_port_bit_number_PGM[arduinoPin]);
    if (portNumber==PB) {
      functionPointerArrayPORTB[portBitNumber] = userFunction;
      portSnapshotB=*portInputRegister(portNumber);
    }
    if (portNumber==PC) {
      functionPointerArrayPORTC[portBitNumber] = userFunction;
      printPSTR("function pointer array entry number: 0x");
      Serial.println(portBitNumber, HEX);
      portSnapshotC=*portInputRegister(portNumber); // OK-MIKE
    }
    if (portNumber==PD) {
      functionPointerArrayPORTD[portBitNumber] = userFunction;
      portSnapshotD=*portInputRegister(portNumber); // OK-MIKE
    }
    // set the appropriate bit on the appropriate PCMSK register
    pcmsk=digitalPinToPCMSK(arduinoPin);        // appropriate PCMSK register
    *pcmsk |= portMask;  // appropriate bit, e.g. this could be PCMSK1 |= portMask;

    // With the exception of the Global Interrupt Enable bit in SREG, interrupts on the arduinoPin
    // are now ready. GIE may have already been set on a previous enable, so it's important
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


/* For Mega 2560
 * #define PORTB_VECT PCINT0_vect
 * #define PORTJ_VECT PCINT1_vect
 * #define PORTK_VECT PCINT2_vect
 *
 */
/* For Leonardo (16u4)
 * #define PORTB_VECT PCINT0_vect
 */

volatile uint8_t functionCalled=0;
volatile uint16_t interruptsCalled=0;
volatile uint8_t risingPins=0;
volatile uint8_t fallingPins=0;


/*
  : "I" (_SFR_IO_ADDR(PINC)) \
*/

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

// If the compiler does not inline this, things break.
// It's not enough to have just void inlineISR. We need the attribute.
//inline void inlineISR (uint8_t current,
void inlineISR (uint8_t current,
//inline void __attribute__((always_inline)) inlineISR (uint8_t current,
  volatile uint8_t *portSnapshot,
  uint8_t risingPins, uint8_t fallingPins,
  volatile uint8_t pcmsk,
  interruptFunctionType functionPointerArray[]) {

  uint8_t i;
  uint8_t interruptMask;
  uint8_t changedPins;

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
  register uint8_t current asm("r18");

//  EI_ASM_PREFIX(PINB);
  current=PINB;

  inlineISR(current,
      &portSnapshotB,
      risingPinsPORTB,
      fallingPinsPORTB,
      PCMSK0,
      functionPointerArrayPORTB );

//  EI_ASM_SUFFIX;
}

ISR(PORTC_VECT) {
//  ISR(PORTC_VECT, ISR_NAKED) {
  register uint8_t current asm("r18");

//  EI_ASM_PREFIX(PINC);
  current=PINC;

  inlineISR(current,
      &portSnapshotC,
      risingPinsPORTC,
      fallingPinsPORTC,
      PCMSK1,
      functionPointerArrayPORTC );

//  EI_ASM_SUFFIX;
}

volatile uint8_t isr_queue[16]={ 0 };
volatile uint8_t portD_IQ=0;
ISR(PORTD_VECT, ISR_NAKED) {
  register uint8_t qPointer=portD_IQ;
  if (qPointer & 0x10) qPointer=0;
  *(isr_queue+qPointer)=PIND;
  qPointer++;
  portD_IQ=qPointer;
}

/*
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
*/
