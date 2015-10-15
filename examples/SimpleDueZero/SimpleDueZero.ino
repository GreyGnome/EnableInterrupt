// EnableInterrupt Simple example sketch. Demonstrates operation on a single pin of your choice.
// See https://github.com/GreyGnome/EnableInterrupt and the README.md for more information.
#include <EnableInterrupt.h>

// Modify this at your leisure. Refer to https://github.com/GreyGnome/EnableInterrupt/wiki/Usage#Summary
#define ARDUINOPIN 10

volatile uint16_t interruptCount=0; // The count will go back to 0 after hitting 65535.

void interruptFunction() {
  interruptCount++;
}

void setup() {
  Serial.begin(115200);
  pinMode(ARDUINOPIN, INPUT_PULLUP);  // See http://arduino.cc/en/Tutorial/DigitalPins
  enableInterrupt(ARDUINOPIN, interruptFunction, CHANGE);
}

// In the loop we just display interruptCount. The value is updated by the interrupt routine.
void loop() {
  Serial.println("---------------------------------------");
  delay(1000);
  Serial.print("Pin was interrupted: ");
  Serial.print(interruptCount, DEC);
  Serial.println(" times so far.");
}

//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// GORY DETAILS //////////////////////////////////////

/* The EnableInterrupt library was designed to present a single API for interrupts to all
 * the Arduino processors. The problems have traditionally been caused by the varying and
 * complicated pin interrupt schemes of the ATmega line of processors. The SAM processors
 * present a more straightforward interrupt design. They already do what the EnableInterrupt
 * library is trying to present: To enable an interrupt on a pin, simply set up that pin
 * with the type of interrupt you want, no conversion to an "Interrupt Number" [see the
 * attachInterrupt() docs].
 *
 * This means that the EnableInterrupt library's work on the Due and Zero is trivial: if
 * called on those platforms, we simply create a macro that converts enableInterrupt() and
 * disableInterrupt() to attachInterrupt()/detachInterrupt().
 *
 * Since that's all it does, the ONLY advantage the EnableInterrupt library offers is as
 * a consistent API across the chip families. If you are not interested in this then don't use
 * the EnableInterrupt library.
 */
