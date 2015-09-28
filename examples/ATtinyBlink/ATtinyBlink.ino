#include <EnableInterrupt.h>

#ifdef EI_ATTINY24
#define OUTPUTPIN 10     // == B0
#define INTERRUPTEDPIN 9 // == B1
// Modify this at your leisure. But you must be aware of which port it's on (see below).
#elif defined EI_ATTINY25
#define OUTPUTPIN 0      // == B0
#define INTERRUPTEDPIN 1 // == B1
#else
#error Dang, only ATtiny chips supported by this sketch.
#endif

// Connect a LED to Pin 3. It might be different in different ATtiny micro controllers
//const uint8_t outputBitMask = 0x01;
const uint8_t outputBitMask = digital_pin_to_bit_mask_PGM[OUTPUTPIN];
const uint8_t inputBitMask = digital_pin_to_bit_mask_PGM[INTERRUPTEDPIN];

volatile uint16_t endlessCounter=0; // The count will go back to 0 after hitting 65535.
volatile uint8_t interruptState=0;

void interruptFunction() {
  if (interruptState) {
    interruptState=0;
    PORTB &= ~_BV(outputBitMask);
  } else {
    interruptState=1;
    PORTB |= 1 << outputBitMask;
  }
}

// the setup routine runs once when you press reset:
void setup() {
  DDRB |= outputBitMask; // OUTPUTPIN is set to output
  DDRB &= ~inputBitMask; // INPUTPIN is set to input
  PORTB |= inputBitMask; // Pull up the resistor
  MCUCR &= ~PUD; // ensure Pull Up Disable is off
  enableInterrupt(INTERRUPTEDPIN, interruptFunction, CHANGE);
}
 
// the loop routine runs over and over again forever:
void loop() {
  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(1000);               // wait for a second
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  //delay(1000);               // wait for a second
  endlessCounter++; // give it something to do
}
