// EnableInterrupt Simple example sketch
// See https://github.com/GreyGnome/EnableInterrupt and the README.md for more information.
//
// https://github.com/GreyGnome/EnableInterrupt/wiki/Usage#Summary

#define NEEDFORSPEED

#define NOTIFY A5

#define PIN8 8
#define PIN8_ON PORTB |= (1 << PB0)
#define PIN8_OFF PORTB &= ~(1 << PB0)
#ifdef NEEDFORSPEED
#define INTERRUPT_FLAG_PIN8 myvariable_pin8 // NOTICE: NO semicolon!!!
#else
volatile uint8_t myvariable_pin8=0;
#endif

#include <EnableInterrupt.h>

void incrementMyVariable() { // The compiler is smart: only uses memory if needed (ie, NEEDFORSPEED not defined)
  myvariable_pin8++;
}

// Attach the interrupt in setup()
void setup() {
  //uint8_t pind, pink;
  Serial.begin(115200);
  EI_printPSTR("---------------------------------------\r\n");
  pinMode(PIN8, INPUT_PULLUP);  // Configure the pin as an input, and turn on the pullup resistor.
                                      // See http://arduino.cc/en/Tutorial/DigitalPins
  pinMode(NOTIFY, OUTPUT);  // Configure the pin as an output
#ifdef NEEDFORSPEED
  enableInterruptFast(PIN8, CHANGE);
#else
  enableInterrupt(PIN8, incrementMyVariable, CHANGE);
#endif
}

// In the loop, we just check to see where the interrupt count is at. The value gets updated by the
// interrupt routine.
void loop() {

  EI_printPSTR("---------------------------------------\r\n");
  delay(1000);                          // Every second,
  PIN8_ON; 	// software interrupt
  PORTB |= (1 << PB0); // MIKE TEST DEBUG
  PORTC |= (1 << PC5);  // == A5. We will enter the interrupt.
  //PORTC=0x01;
  //PORTC=0x00; // software interrupt, port A0
  delay(2000); // Every second,
  PORTC |= (1 << PC5);  // == A5. We have left the interrupt.
  PORTC &= ~(1 << PC5); // == A5. 
  if (myvariable_pin8) {
    EI_printPSTR("Pin was interrupted: ");
    Serial.print(myvariable_pin8, DEC);      // print the interrupt count.
    EI_printPSTR(" times so far.\r\n");
    myvariable_pin8=0;
  }
  else {
    EI_printPSTR("No interrupts.\r\n");
  }
}
