#ifdef EI_SECTION_RISING
#ifndef EI_NOTPORTB
  risingPinsPORTB |= portMask;
#endif
#endif // EI_SECTION_RISING

#ifdef EI_SECTION_FALLING
#ifndef EI_NOTPORTB
  fallingPinsPORTB |= portMask;
#endif
#endif // EI_SECTION_FALLING

#if defined EI_SECTION_ASSIGNFUNCTIONSREGISTERS
#ifndef EI_NOTPORTB
#ifndef NEEDFORSPEED
  calculatedPointer=&portBFunctions.pinZero + portBitNumber;
  *calculatedPointer = userFunction;
#endif
  portSnapshotB=*portInputRegister(portNumber);
  pcmsk=&PCMSK;
  GIMSK |= _BV(5);
#endif
#endif // EI_SECTION_ASSIGNFUNCTIONSREGISTERS

#ifdef EI_SECTION_DISABLEPINCHANGE
#ifndef EI_NOTPORTB
  PCMSK &= ~portMask;
  if (PCMSK == 0) { GIMSK &= ~_BV(5); };
  risingPinsPORTB &= ~portMask;
  fallingPinsPORTB &= ~portMask;
#endif
#endif // EI_SECTION_DISABLEPINCHANGE
