// Support for the 14-pin ATtiny24/24A/44/44A/84/84A
#if ! defined(EI_NOTINT0)
#ifdef EI_SECTION_ENABLEEXTERNAL
// NO switch (arduinoPin) { // ONLY 1 External Interrupt pin.
#ifndef EI_NOTINT0
    GIMSK &= ~_BV(6); // Disable interrupts since we are (possibly) changing interrupt settings
#ifndef NEEDFORSPEED
    externalFunctionPointer = userFunction;
#endif
    MCUCR &= (~_BV(0) & ~_BV(1)); // reset the flags prior to
    MCUCR |= mode;                // set them the way we want
    GIFR |= _BV(6);
    GIMSK |= _BV(6);
#endif
#endif // EI_SECTION_ENABLEEXTERNAL

#ifdef EI_SECTION_DISABLEEXTERNAL
#ifndef EI_NOTINT0
  GIMSK &= ~_BV(6);
  MCUCR &= (~_BV(0) & ~_BV(1)); // reset the flags
  GIFR  |= _BV(6); // using a clue from the ATmega2560 datasheet.
#endif
#endif // EI_SECTION_DISABLEEXTERNAL
#endif // ! defined(EI_NOTINT0) && ! defined (EI_NOTINT1) && ! defined (EI_NOTINT2)
