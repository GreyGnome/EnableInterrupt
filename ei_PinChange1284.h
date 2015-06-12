#ifdef EI_SECTION_RISING
if (portNumber==PA) {
  risingPinsPORTA |= portMask;
}
if (portNumber==PB) {
  risingPinsPORTB |= portMask;
}
if (portNumber==PC) {
  risingPinsPORTC |= portMask;
}
if (portNumber==PD) {
  risingPinsPORTD |= portMask;
}
#endif
#ifdef EI_SECTION_FALLING
if (portNumber==PA) {
  fallingPinsPORTA |= portMask;
}
if (portNumber==PB) {
  fallingPinsPORTB |= portMask;
}
if (portNumber==PC) {
  fallingPinsPORTC |= portMask;
}
if (portNumber==PD) {
  fallingPinsPORTD |= portMask;
}
#endif
#if defined EI_SECTION_ASSIGNFUNCTIONSREGISTERS
if (portNumber==PA) {
#ifndef NEEDFORSPEED
  calculatedPointer=&portAFunctions.pinZero + portBitNumber;
  *calculatedPointer = userFunction;
#endif
  portSnapshotA=*portInputRegister(portNumber);
  pcmsk=&PCMSK0;
  PCICR |= _BV(0);
}
if (portNumber==PB) {
#ifndef NEEDFORSPEED
  calculatedPointer=&portBFunctions.pinZero + portBitNumber;
  *calculatedPointer = userFunction;
#endif
  portSnapshotB=*portInputRegister(portNumber);
  pcmsk=&PCMSK1;
  PCICR |= _BV(1);
}
if (portNumber==PC) {
#ifndef NEEDFORSPEED
  calculatedPointer=&portCFunctions.pinZero + portBitNumber;
  *calculatedPointer = userFunction;
#endif

  portSnapshotC=*portInputRegister(portNumber);
  pcmsk=&PCMSK2;
  PCICR |= _BV(2);
}
if (portNumber==PD) {
#ifndef NEEDFORSPEED
  calculatedPointer=&portDFunctions.pinZero + portBitNumber;
  *calculatedPointer = userFunction;
#endif

  portSnapshotD=*portInputRegister(portNumber);
  pcmsk=&PCMSK3;
  PCICR |= _BV(3);
}
#endif

#ifdef EI_SECTION_DISABLEPINCHANGE
if (portNumber == PA) {
  PCMSK0 &= ~portMask;
  if (PCMSK0 == 0) { PCICR &= ~_BV(0); };
  risingPinsPORTA &= ~portMask;
  fallingPinsPORTA &= ~portMask;
}
if (portNumber == PB) {
  PCMSK1 &= ~portMask;
  if (PCMSK1 == 0) { PCICR &= ~_BV(1); };
  risingPinsPORTB &= ~portMask;
  fallingPinsPORTB &= ~portMask;
}
if (portNumber == PC) {
  PCMSK2 &= ~portMask;
  if (PCMSK2 == 0) { PCICR &= ~_BV(2); };
  risingPinsPORTC &= ~portMask;
  fallingPinsPORTC &= ~portMask;
}
if (portNumber == PD) {
  PCMSK3 &= ~portMask;
  if (PCMSK3 == 0) { PCICR &= ~_BV(3); };
  risingPinsPORTB &= ~portMask;
  fallingPinsPORTB &= ~portMask;
}
#endif

#ifdef EI_SECTION_DISABLEEXTERNAL
if (arduinoPin == 10) {
  EIMSK &= ~_BV(0);
  EICRA &= (~_BV(0) & ~_BV(1));
  EIFR  |= _BV(0); // using a clue from the ATmega2560 datasheet.
}
if (arduinoPin == 11) {
  EIMSK &= ~_BV(1);
  EICRA &= (~_BV(2) & ~_BV(3));
  EIFR  |= _BV(1); // using a clue from the ATmega2560 datasheet.
}
if (arduinoPin == 2) {
  EIMSK &= ~_BV(2);
  EICRA &= (~_BV(4) & ~_BV(5));
  EIFR  |= _BV(2); // using a clue from the ATmega2560 datasheet.
}
#endif
