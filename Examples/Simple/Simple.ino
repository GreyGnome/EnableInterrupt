// EnableInterrupt Simple example sketch
// See the Wiki at http://code.google.com/p/arduino-pinchangeint/wiki for more information.

// for vim editing: :set et ts=2 sts=2 sw=2 et

// This example demonstrates the use of the EnableInterrupt library on a single pin of your choice.
// This has only been tested on an Arduino Duemilanove and Mega ADK.
// To use:

// 1. You must be using a fairly recent version of the Arduino IDE software on your PC/Mac,
// that is, version 1.0.1 or later. Check Help->About Arduino in the IDE.

// 2. Wire a simple switch to any Analog or Digital pin (known as ARDUINOPIN, defined below).
// Attach the other end to a GND pin. A "single pole single throw momentary contact"
// pushbutton switch is best for the best interrupting fun.

// 3. When pressed, the switch will connect the pin to ground ("low", or "0") voltage, and interrupt the
// processor. Don't let it confuse you that a switch press means the pin's voltage goes to 0; it
// may seem more intuitive to apply a "1" or high voltage to the pin to represent "pressed".
// But the processor is perfectly happy that we've made "0" equal "Pressed". The reason we've done so
// is because we are using the "internal pullup resistor" feature of the processor... the chip gives
// us a free resistor on every pin!
// See http://arduino.cc/en/Tutorial/DigitalPins for a complete explanation.

// 4. The interrupt is serviced immediately, and the ISR (Interrupt SubRoutine) sets the value of a global
// variable. The sketch can then query the value at its leisure. This makes loop timing non-critical.
// Open Tools->Serial Monitor in the IDE to see the results of your interrupts.

// 5. See PinChangeIntExample328.ino (in the PinChangeInt distribution) for a more elaborate example.

// 6. Create your own sketch using the PinChangeInt library!

#include <EnableInterrupt.h>
//#include <PinChangeInt.h>

// Modify this at your leisure.
// SS = PB0, A8 = PK0, 15 = PJ0, 14 = PJ1
//#define ARDUINOPIN A10
#define ARDUINOPIN 3
//#define ARDUINOPIN 14

// Do not use any Serial.print() in interrupt subroutines. Serial.print() uses interrupts,
// and by default interrupts are off in interrupt subroutines. Interrupt routines should also
// be as fast as possible. Here we just increment a counter.
volatile uint16_t interruptCount=0; // The count will go back to 0 after hitting 65535.

void interruptFunction() {
  interruptSaysHello();
  interruptCount++;
}
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
void setup() {
  //uint8_t pind, pink;
  Serial.begin(115200);
  Serial.println("---------------------------------------");
  //PORTD=pind;
  //PORTK=pink;
  pinMode(PINSIGNAL, OUTPUT);
  pinMode(ARDUINOPIN, INPUT_PULLUP);  // Configure the pin as an input, and turn on the pullup resistor.
                                      // See http://arduino.cc/en/Tutorial/DigitalPins
  //PORTC=0x01;
  pinMode(SS, INPUT_PULLUP);
  pinMode(SCK, INPUT_PULLUP);
  pinMode(MOSI, INPUT_PULLUP);
  pinMode(MISO, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(ARDUINOPIN, INPUT_PULLUP);
  enableInterrupt(ARDUINOPIN, interruptFunction, CHANGE);
  //attachPinChangeInterrupt(ARDUINOPIN, interruptFunction, CHANGE);
  //current=PINC;
  interruptSaysHello(); // to turn on Signal (LED on 328-based Arduino, 21 on Mega)
}

// In the loop, we just check to see where the interrupt count is at. The value gets updated by the
// interrupt routine.
void loop() {
  uint8_t led_on, led_off;         // DEBUG

  led_on=0b00100000; led_off=~led_on;

  //*led_port|=led_mask; // LED high
  //PORTB|=led_on;
  //PORTC=0x01;
  Serial.println("---------------------------------------");
  delay(1000);                          // Every second,
  //PORTC=0x00; // software interrupt, port A0
  //PORTB|=led_on;
  //PORTB&=led_off;
  //PORTB|=led_on;
  //PORTB&=led_off;
  //PORTB|=led_on;

  //PORTB&=led_off;
  //PORTB|=led_on;
  Serial.print("Pin was interrupted: ");
  Serial.print(interruptCount, DEC);      // print the interrupt count.
  Serial.println(" times so far.");

  //Serial.print(" isr's: ");
  //Serial.print(interruptsCalled, DEC);
  //Serial.print(" current: 0x");
  //Serial.print(current, HEX);
  //Serial.print(" changedPins: 0x");
  //Serial.print(changedPins, HEX);
  //Serial.print(" function: ");
  //Serial.println(functionCalled, DEC);
  //functionCalled=0;
}
