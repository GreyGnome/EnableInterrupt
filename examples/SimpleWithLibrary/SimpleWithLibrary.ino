// SimpleWithLibrary example sketch for the EnableInterrupt library. Demonstrates how to use
// the EnableInterrupt library with a sketch that services an interrupt pin, and uses a library
// that also requires the EnableInterrupt library.

// This sketch has only been compiled to the Arduino Uno (ATmega328).
// This sketch is only known to compile. It has not been tested functionally.

// See https://github.com/GreyGnome/EnableInterrupt and the README.md for more information.
#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>
#define USELESSPIN 10
#include "Useless.h"

#define ARDUINOPIN 9

volatile uint16_t interruptCount=0;

void interruptFunction() {
  interruptCount++;
}

// UselessClass uses the EnableInterrupt library.
UselessClass uselessObject=UselessClass(USELESSPIN, CHANGE);

void setup() {
  Serial.begin(115200);
  pinMode(ARDUINOPIN, INPUT_PULLUP);  // See http://arduino.cc/en/Tutorial/DigitalPins
  enableInterrupt(ARDUINOPIN, interruptFunction, CHANGE);
}

void loop() {
  Serial.println("---------------------------------------");
  delay(1000);
  Serial.print("Pin was interrupted: ");
  Serial.print(interruptCount, DEC);
  Serial.println(" times so far.");
  Serial.print("Useless interrupt count: ");
  Serial.print(uselessObject.getUselessVariable(), DEC);
}
