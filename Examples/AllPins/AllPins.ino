// EnableInterrupt Simple example sketch
// See the Wiki at http://code.google.com/p/arduino-pinchangeint/wiki for more information.

// This example demonstrates the use of the EnableInterrupt library on all pins.
// This has only been tested on an Arduino Duemilanove and Mega ADK.
// To use:

#include <EnableInterrupt.h>

volatile uint8_t anyInterruptCounter=0;

#ifdef ARDUINO_MEGA

#define PINCOUNT(x) pin ##x ##Count

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
    PINCOUNT(x)=0; \
  }

#define setupInterrupt(x) \
  pinMode( x, INPUT_PULLUP); \
  enableInterrupt( x, interruptFunction##x, CHANGE)

interruptFunction(SS);
interruptFunction(SCK);
interruptFunction(MOSI);
interruptFunction(MISO);
interruptFunction(10);
interruptFunction(11);
interruptFunction(12);
interruptFunction(13);
interruptFunction(14);
interruptFunction(15);
interruptFunction(A8);
interruptFunction(A9);
interruptFunction(A10);
interruptFunction(A11);
interruptFunction(A12);
interruptFunction(A13);
interruptFunction(A14);
interruptFunction(A15);
interruptFunction(70); // fake 70, trick to allow software interrupts on Port J. PJ2
interruptFunction(71); // fake 71. PJ3
interruptFunction(72); // fake 72. PJ4
interruptFunction(73); // fake 73. PJ5
interruptFunction(74); // fake 74. PJ6
#endif

void printIt(char *pinNumber, uint8_t count) {
    printPSTR("Pin ");
    Serial.print(pinNumber);
    printPSTR(" was interrupted: ");
    Serial.println(count, DEC);
}

// Do not use any Serial.print() in interrupt subroutines. Serial.print() uses interrupts,
// and by default interrupts are off in interrupt subroutines. Interrupt routines should also
// be as fast as possible. Here we just increment a counter.
volatile uint16_t interruptCount=0; // The count will go back to 0 after hitting 65535.

/*
void interruptFunction() {
  interruptSaysHello();
  interruptCount++;
}
*/

// Notice that values that get modified inside an interrupt, that I wish to access
// outside the interrupt, are marked "volatile". It tells the compiler not to optimize
// the variable.

/*
volatile uint16_t interruptCount1=0; // The count will go back to 0 after hitting 65535.
void interruptFunction2() {
  interruptCount1++;
}
*/

/*
//extern volatile uint8_t current;
extern volatile uint8_t risingPinsPORTC;
extern volatile uint8_t fallingPinsPORTC;
extern volatile uint8_t functionCalled;
//extern volatile uint16_t interruptsCalled; // DEBUG
extern volatile uint8_t changedPins;
extern volatile uint8_t risingPins;
extern volatile uint8_t fallingPins;
*/


// Attach the interrupt in setup()
// NOTE: PORTJ2-6 (aka, "Pin '70', '71', '72', '73', '74'" are turned on as OUTPUT.
// These are not true pins on the Arduino Mega series!
void setup() {
  //uint8_t pind, pink;
  Serial.begin(115200);
  Serial.println("---------------------------------------");
  //PORTD=pind;
  //PORTK=pink;
  pinMode(PINSIGNAL, OUTPUT);
  //pinMode(ARDUINOPIN, INPUT_PULLUP);  // Configure the pin as an input, and turn on the pullup resistor.
                                      // See http://arduino.cc/en/Tutorial/DigitalPins
  //PORTC=0x01;
  //setupInterrupt(SS);
  //setupInterrupt(SCK);
  pinMode(SS, INPUT_PULLUP);
  enableInterrupt(SS,  interruptFunctionSS, CHANGE);
  pinMode(SCK, INPUT_PULLUP);
  enableInterrupt(SCK,  interruptFunctionSCK, CHANGE);
  pinMode(MOSI, INPUT_PULLUP);
  enableInterrupt(MOSI, interruptFunctionMOSI, CHANGE);
  pinMode(MISO, INPUT_PULLUP);
  enableInterrupt(MISO, interruptFunctionMISO, CHANGE);
  pinMode(10, INPUT_PULLUP);
  enableInterrupt(10, interruptFunction10, CHANGE);
  pinMode(11, INPUT_PULLUP);
  enableInterrupt(11, interruptFunction11, CHANGE);
  pinMode(12, INPUT_PULLUP);
  enableInterrupt(12, interruptFunction12, CHANGE);
  pinMode(13, INPUT_PULLUP);
  enableInterrupt(13, interruptFunction13, CHANGE);
  pinMode(14, INPUT_PULLUP);
  enableInterrupt(14, interruptFunction14, CHANGE);
  pinMode(15, INPUT_PULLUP);
  enableInterrupt(15, interruptFunction15, CHANGE);
  pinMode(A8, INPUT_PULLUP);
  enableInterrupt(A8, interruptFunctionA8, CHANGE);
  pinMode(A9, INPUT_PULLUP);
  enableInterrupt(A9, interruptFunctionA9, CHANGE);
  pinMode(A10, INPUT_PULLUP);
  enableInterrupt(A10, interruptFunctionA10, CHANGE);
  pinMode(A11, INPUT_PULLUP);
  enableInterrupt(A11, interruptFunctionA11, CHANGE);
  pinMode(A12, INPUT_PULLUP);
  enableInterrupt(A12, interruptFunctionA12, CHANGE);
  pinMode(A13, INPUT_PULLUP);
  enableInterrupt(A13, interruptFunctionA13, CHANGE);
  pinMode(A14, INPUT_PULLUP);
  enableInterrupt(A14, interruptFunctionA14, CHANGE);
  pinMode(A15, INPUT_PULLUP);
  enableInterrupt(A15, interruptFunctionA15, CHANGE);
  /*
  //DDRJ|= 0b11111100; // == 0x40.
  //PORTJ|=0b11111100;
  */
  DDRJ |=0b01111100; // Non-Arduino Port J pins all become output.
  PORTJ|=0b01111100; // Turn them all high.
  enableInterrupt(70, interruptFunction70, CHANGE);
  enableInterrupt(71, interruptFunction71, CHANGE);
  enableInterrupt(72, interruptFunction72, CHANGE);
  enableInterrupt(73, interruptFunction73, CHANGE);
  enableInterrupt(74, interruptFunction74, CHANGE);
  //attachPinChangeInterrupt(ARDUINOPIN, interruptFunction, CHANGE);
  //current=PINC;
  //interruptSaysHello(); // to turn on Signal (LED on 328-based Arduino, 21 on Mega)
}

// In the loop, we just check to see where the interrupt count is at. The value gets updated by the
// interrupt routine.
void loop() {
  //Serial.print(" isr's: ");
  //Serial.print(interruptsCalled, DEC);
  //Serial.print(" current: 0x");
  //Serial.print(current, HEX);
  //Serial.print(" changedPins: 0x");
  //Serial.print(changedPins, HEX);
  //Serial.print(" function: ");
  //Serial.println(functionCalled, DEC);
  //functionCalled=0;
  //digitalWrite(14, LOW);
  //digitalWrite(14, HIGH);
  //uint8_t port=digitalPinToPort(14);
  //volatile uint8_t *out=portOutputRegister(port);
  //uint8_t bit=digitalPinToBitMask(14);
  uint8_t jbits =0b01111110; // PJ2
  uint8_t njbits=0b00000001; // PJ2
  PORTJ &= njbits;
  //*out &= njbits;
  delay(1);
  PORTJ |= jbits;
  //*out |= jbits;
  delay(1);
  //uint8_t jbits=0b11100000;
  //PORTJ&=njbits; // PJn off to trigger interrupt
  //PORTJ|=jbits; // PJn on to trigger interrupt
  /*
  for (uint8_t jctr=2; jctr < 7; jctr++) {
    for (uint8_t j=0; j < jctr; j++) {
      PORTJ&=~jbits; // PJn off to trigger interrupt
      PORTJ|=jbits; // PJn on to trigger interrupt
    }
    jbits<<=1;
  } // MIKE CHECK THIS FEATURE...!
  */

  Serial.println("---------------------------------------");
  delay(1000);                          // Every second,
  updateOn(SS);
  updateOn(SCK);
  updateOn(MOSI);
  updateOn(MISO);
  updateOn(10);
  updateOn(11);
  updateOn(12);
  updateOn(13);
  updateOn(14);
  updateOn(15);
  updateOn(A8);
  updateOn(A9);
  updateOn(A10);
  updateOn(A11);
  updateOn(A12);
  updateOn(A13);
  updateOn(A14);
  updateOn(A15);
  updateOn(70);
  updateOn(71);
  updateOn(72);
  updateOn(73);
  updateOn(74);
  printIt("XXX", anyInterruptCounter);
  /*if (pinSSCount != 0) {
    Serial.print("Pin was interrupted: ");
    Serial.print(pinSSCount, DEC);      // print the interrupt count.
    Serial.println(" times over the last second.");
    pinSSCount=0;
  }*/

}

