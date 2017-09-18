/*
 *  This example shows how to combine the EnableInterrupt
 *  library with some simple debouncing code for reading
 *  button presses consistently.
 *  
 *  The interrupt is attached to digital input 5 and the
 *  service routine just toggles the onboard LED status.
 *  
 *  Tested on: ATmega328P
 *  
 *  Example by Lorenzo Cafaro <lorenzo@ibisco.net>
 *
*/

#include <EnableInterrupt.h>

#define BUTTON_PIN 5
#define DEBOUNCE_DELAY 100 // in ms

uint32_t last_interrupt_time = 0;
uint8_t led_status = 0;

void setup() {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  enableInterrupt(BUTTON_PIN, isr_handler, RISING);
}

void loop() {
  // zZz
}

void isr_handler() {
  uint32_t interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY) {
    led_status = !led_status;
    digitalWrite(LED_BUILTIN, led_status);
  }

  last_interrupt_time = interrupt_time;
}
