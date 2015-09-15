// ALL pin change interrupts are on Port B on Leonardo.
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
pcmsk=&PCMSK0;
PCICR |= _BV(0);
#endif
#endif // EI_SECTION_ASSIGNFUNCTIONSREGISTERS

#ifdef EI_SECTION_DISABLEPINCHANGE
#ifndef EI_NOTPORTB
PCMSK0 &= ~portMask;
if (PCMSK0 == 0) { PCICR &= ~_BV(0); };
risingPinsPORTB &= ~portMask;
fallingPinsPORTB &= ~portMask;
#endif
#endif // EI_SECTION_DISABLEPINCHANGE
