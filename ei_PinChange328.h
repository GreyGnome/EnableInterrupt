#ifdef EI_SECTION_RISING
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
if (portNumber==PB) {
#ifndef NEEDFORSPEED
  calculatedPointer=&portBFunctions.pinZero + portBitNumber;
  *calculatedPointer = userFunction;
#endif
  portSnapshotB=*portInputRegister(portNumber);
  pcmsk=&PCMSK0;
  PCICR |= _BV(0);
}
if (portNumber==PC) {
#ifndef NEEDFORSPEED
  calculatedPointer=&portCFunctions.pinZero + portBitNumber;
  *calculatedPointer = userFunction;
#endif

  portSnapshotC=*portInputRegister(portNumber);
  pcmsk=&PCMSK1;
  PCICR |= _BV(1);
}
if (portNumber==PD) {
#ifndef NEEDFORSPEED
  calculatedPointer=&portDFunctions.pinZero + portBitNumber;
  *calculatedPointer = userFunction;
#endif

  portSnapshotD=*portInputRegister(portNumber);
  pcmsk=&PCMSK2;
  PCICR |= _BV(2);
}
#endif

#ifdef EI_SECTION_DISABLEPINCHANGE
if (portNumber == PB) {
  PCMSK0 &= ~portMask;
  if (PCMSK0 == 0) { PCICR &= ~_BV(0); };
  risingPinsPORTB &= ~portMask;
  fallingPinsPORTB &= ~portMask;
}
if (portNumber == PC) {
  PCMSK1 &= ~portMask;
  if (PCMSK1 == 0) { PCICR &= ~_BV(1); };
  risingPinsPORTC &= ~portMask;
  fallingPinsPORTC &= ~portMask;
}
if (portNumber == PD) {
  PCMSK2 &= ~portMask;
  if (PCMSK2 == 0) { PCICR &= ~_BV(2); };
  risingPinsPORTD &= ~portMask;
  fallingPinsPORTD &= ~portMask;
}
#endif
