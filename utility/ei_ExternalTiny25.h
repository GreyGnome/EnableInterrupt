// Support for the 8-pin ATtiny25/45/85
#ifdef EI_SECTION_ENABLEEXTERNAL
// NO switch (arduinoPin) { // ONLY 1 External Interrupt pin.
#ifndef EI_NOTINT0
    GIMSK &= ~_BV(6);
#ifndef NEEDFORSPEED
    externalFunctionPointer = userFunction;
#endif
    MCUCR &= (~_BV(0) & ~_BV(1)); // reset the flags prior to
    MCUCR |= mode;                // set them the way we want
    GIFR |= _BV(6);
    GIMSK |= _BV(6);
#endif // ! defined (EI_NOTINT0)
#endif // EI_SECTION_ENABLEEXTERNAL

#ifdef EI_SECTION_DISABLEEXTERNAL
#ifndef EI_NOTINT0
  GIMSK &= ~_BV(6);
  GIFR  |= _BV(6); // using a clue from the ATmega2560 datasheet.
  MCUCR &= (~_BV(0) & ~_BV(1));
#endif
#endif // EI_SECTION_DISABLEEXTERNAL
