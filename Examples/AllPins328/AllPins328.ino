// EnableInterrupt Simple example sketch
// See the Wiki at http://code.google.com/p/arduino-pinchangeint/wiki for more information.

// This example demonstrates the use of the EnableInterrupt library on all pins.
// This has only been tested on an Arduino Duemilanove and Mega ADK.
// To use:

#define SHOWEXTERNAL
volatile uint8_t wasExternalInterrupt=0;
#include <EnableInterrupt.h>

volatile uint8_t anyInterruptCounter=0;

#ifdef ARDUINO_328
#define PINCOUNT(x) pin ##x ##Count

// Do not use any Serial.print() in interrupt subroutines. Serial.print() uses interrupts,
// and by default interrupts are off in interrupt subroutines. Interrupt routines should also
// be as fast as possible. Here we just increment counters.
#define interruptFunction(x) \
  volatile uint8_t PINCOUNT(x); \
  void interruptFunction ##x () { \
    anyInterruptCounter++; \
    interruptSaysHello(); \
    PINCOUNT(x)++; \
  }

#define updateOn(x) \
  if (PINCOUNT(x) != 0) { \
    printIt((char *) #x, PINCOUNT(x)); \
    if (wasExternalInterrupt > 0) { printPSTR(" ext: "); Serial.println(wasExternalInterrupt); }; \
    PINCOUNT(x)=0; \
  }

#define setupPCInterrupt(x) \
  pinMode( x, INPUT_PULLUP); \
  enableInterrupt( x | PINCHANGEINTERRUPT, interruptFunction##x, CHANGE)

#define setupInterrupt(x) \
  pinMode( x, INPUT_PULLUP); \
  enableInterrupt( x, interruptFunction##x, CHANGE)

interruptFunction(2);
interruptFunction(3);
interruptFunction(4);
interruptFunction(5);
interruptFunction(6);
interruptFunction(7);
interruptFunction(8);
interruptFunction(9);
interruptFunction(10);
interruptFunction(11);
interruptFunction(12);
interruptFunction(13);
interruptFunction(A0);
interruptFunction(A1);
interruptFunction(A2);
interruptFunction(A3);
interruptFunction(A4);
interruptFunction(A5);
#else
#error This sketch supports 328-based Arduinos only.
#endif

void printIt(char *pinNumber, uint8_t count) {
    printPSTR(" Pin ");
    Serial.print(pinNumber);
    printPSTR(" was interrupted: ");
    Serial.println(count, DEC);
}

// Attach the interrupt in setup()
// NOTE: PORTJ2-6 (aka, "Pin '70', '71', '72', '73', '74'" are turned on as OUTPUT.
// These are not true pins on the Arduino Mega series!
void setup() {
  Serial.begin(115200);
  Serial.println("---------------------------------------");
#ifdef DEBUG
  pinMode(PINSIGNAL, OUTPUT);
#endif
  // PINS 0 and 1 NOT USED BECAUSE OF Serial.print()
  setupPCInterrupt(2); // by default, will be External Interrupt
  setupInterrupt(3);
  setupInterrupt(4);
  setupInterrupt(5);
  setupInterrupt(6);
  setupInterrupt(7);
  setupInterrupt(8);
  setupInterrupt(9);
  setupInterrupt(10);
  setupInterrupt(11);
  setupInterrupt(12);
#ifndef DEBUG
  // NOTE: Because the Arduino Duemilanove has an LED to ground and a 1k resistor in series with
  // it to the pin, Voltage at the pin should be hovering between 1-3 volts. 'nearly' ground. So
  // a wire to ground will not trip an interrupt, even though we have INPUT_PULLUP. A wire to PWR
  // will trigger an interrupt. The Uno has a op-amp buffer/driver to the LED, so will not have
  // this problem.
  setupInterrupt(13);
#endif
  setupInterrupt(A0);
  setupInterrupt(A1);
  setupInterrupt(A2);
  setupInterrupt(A3);
  setupInterrupt(A4);
  setupInterrupt(A5);
}

// In the loop, we just check to see where the interrupt count is at. The value gets updated by the
// interrupt routine.
void loop() {
  Serial.println("---------------------------------------");
  delay(1000);                          // Every second,
  updateOn(2);
  updateOn(3);
  updateOn(4);
  updateOn(5);
  updateOn(6);
  updateOn(7);
  updateOn(8);
  updateOn(9);
  updateOn(10);
  updateOn(11);
  updateOn(12);
  updateOn(13);
  updateOn(A0);
  updateOn(A1);
  updateOn(A2);
  updateOn(A3);
  updateOn(A4);
  updateOn(A5);
  printIt((char *) "XXX", anyInterruptCounter);
  wasExternalInterrupt=0;
}

