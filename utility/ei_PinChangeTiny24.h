#ifdef EI_SECTION_RISING
#ifndef EI_NOTPORTA
if (portNumber==PA) {
  risingPinsPORTA |= portMask;
}
#endif
#ifndef EI_NOTPORTB
if (portNumber==PB) {
  risingPinsPORTB |= portMask;
}
#endif
#endif // EI_SECTION_RISING

#ifdef EI_SECTION_FALLING
#ifndef EI_NOTPORTA
if (portNumber==PA) {
  fallingPinsPORTA |= portMask;
}
#endif
#ifndef EI_NOTPORTB
if (portNumber==PB) {
  fallingPinsPORTB |= portMask;
}
#endif
#endif // EI_SECTION_FALLING

#if defined EI_SECTION_ASSIGNFUNCTIONSREGISTERS
#ifndef EI_NOTPORTA
if (portNumber==PA) {
#ifndef NEEDFORSPEED
  calculatedPointer=&portAFunctions.pinZero + portBitNumber;
  *calculatedPointer = userFunction;
#endif
  portSnapshotA=*portInputRegister(portNumber);
  pcmsk=&PCMSK0;
  GIMSK |= _BV(4);
}
#endif
#ifndef EI_NOTPORTB
if (portNumber==PB) {
#ifndef NEEDFORSPEED
  calculatedPointer=&portBFunctions.pinZero + portBitNumber;
  *calculatedPointer = userFunction;
#endif
  portSnapshotB=*portInputRegister(portNumber);
  pcmsk=&PCMSK1;
  GIMSK |= _BV(5);
}
#endif
#endif // EI_SECTION_ASSIGNFUNCTIONSREGISTERS

#ifdef EI_SECTION_DISABLEPINCHANGE
#ifndef EI_NOTPORTA
if (portNumber == PA) {
  PCMSK0 &= ~portMask;
  if (PCMSK0 == 0) { GIMSK &= ~_BV(4); };
  risingPinsPORTA &= ~portMask;
  fallingPinsPORTA &= ~portMask;
}
#endif
#ifndef EI_NOTPORTB
if (portNumber == PB) {
  PCMSK1 &= ~portMask;
  if (PCMSK1 == 0) { GIMSK &= ~_BV(5); };
  risingPinsPORTB &= ~portMask;
  fallingPinsPORTB &= ~portMask;
}
#endif
#endif // EI_SECTION_DISABLEPINCHANGE
