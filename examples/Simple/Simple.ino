// EnableInterrupt Simple example sketch
// See https://github.com/GreyGnome/EnableInterrupt and the README.md for more information.

// This example demonstrates the use of the EnableInterrupt library on a single pin of your choice.
// This has only been tested on an Arduino Duemilanove and Mega ADK.
// It is designed to work with the Arduino Duemilanove/Uno, Arduino Mega2560/ADK, the Arduino
// Leonardo, and the Arduino Due. Please let me know how you fare on the Leonardo or Due.

// To use:

// 1. You must be using a fairly recent version of the Arduino IDE software on your PC/Mac,
// that is, version 1.0.1 or later. Check Help->About Arduino in the IDE.

// 2. Wire a simple switch to any Analog or Digital pin (known as ARDUINOPIN, defined below)
// that supports interrupts. See https://github.com/GreyGnome/EnableInterrupt/wiki/Usage#Summary
// Attach the other end to a GND pin. A "single pole single throw momentary contact normally
// open" // pushbutton switch is best for the most interrupting fun.
// See https://www.sparkfun.com/products/97 and https://octopart.com/b3f-1000-omron-3117

// 3. When pressed, the switch will connect the pin to ground ("low", or "0") voltage, and interrupt the
// processor. See http://arduino.cc/en/Tutorial/DigitalPins

// 4. The interrupt is serviced immediately, and the ISR (Interrupt SubRoutine) sets the value of a global
// variable. Open Tools->Serial Monitor in the IDE to see the results of your interrupts.

// 5. Peruse the Examples directory for more elaborate examples.

// 6. Create your own sketch using the EnableInterrupt library!

#include <EnableInterrupt.h>

// Modify this at your leisure. Refer to
// https://github.com/GreyGnome/EnableInterrupt/wiki/Usage#Summary
#define ARDUINOPIN 8

// Do not use any Serial.print() in interrupt subroutines. Serial.print() uses interrupts,
// and by default interrupts are off in interrupt subroutines. Interrupt routines should also
// be as fast as possible. Here we just increment a counter.
//
// Notice that values that get modified inside an interrupt, that I wish to access
// outside the interrupt, are marked "volatile". It tells the compiler not to optimize
// the variable.
volatile uint16_t interruptCount=0; // The count will go back to 0 after hitting 65535.

void interruptFunction() {
  interruptCount++;
}

// Attach the interrupt in setup()
void setup() {
  //uint8_t pind, pink;
  Serial.begin(115200);
  Serial.println("---------------------------------------");
  pinMode(ARDUINOPIN, INPUT_PULLUP);  // Configure the pin as an input, and turn on the pullup resistor.
                                      // See http://arduino.cc/en/Tutorial/DigitalPins
  enableInterrupt(ARDUINOPIN, interruptFunction, CHANGE);
}

// In the loop, we just check to see where the interrupt count is at. The value gets updated by the
// interrupt routine.
void loop() {
  Serial.println("---------------------------------------");
  delay(1000);                          // Every second,
  Serial.print("Pin was interrupted: ");
  Serial.print(interruptCount, DEC);      // print the interrupt count.
  Serial.println(" times so far.");
}
