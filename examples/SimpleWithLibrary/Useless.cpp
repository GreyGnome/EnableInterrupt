// FOR TESTING, TO DEMONSTRATE HOW TO USE ENABLEINTERRUPT IN A LIBRARY.
// See Useless.h for more information.
#include "Useless.h"

volatile uint8_t uselessVariable=0;

void uselessFunction() {
	uselessVariable++;
}

void UselessClass::init(uint8_t pin, uint8_t mode) {
	pinMode(pin, INPUT_PULLUP);
	enableInterrupt(pin, uselessFunction, mode);
}

uint8_t UselessClass::getUselessVariable() {
	return uselessVariable;
}

void UselessClass::reset() {
	uselessVariable=0;
}

void UselessClass::disable(uint8_t pin) {
	disableInterrupt(pin);
}
