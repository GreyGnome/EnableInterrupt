#ifdef EI_SECTION_RISING
if (portNumber==PB) { 
  risingPinsPORTB |= portMask;
}
#endif

#ifdef EI_SECTION_FALLING
if (portNumber==PB) { 
  fallingPinsPORTB |= portMask;
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
#endif

#ifdef EI_SECTION_DISABLEPINCHANGE
if (portNumber == PB) {
  PCMSK0 &= ~portMask;
  if (PCMSK0 == 0) { PCICR &= ~_BV(0); };
  risingPinsPORTB &= ~portMask;
  fallingPinsPORTB &= ~portMask;
}
#endif
