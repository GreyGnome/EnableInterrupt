#if ! defined(EI_NOTINT0) && ! defined (EI_NOTINT1) && ! defined (EI_NOTINT2)
#ifdef EI_SECTION_ENABLEEXTERNAL
switch (arduinoPin) {
#ifndef EI_NOTINT0
  case ARDUINO_PIN_D2 : // INT0  (Either 2 or 10)
    EIMSK &= ~_BV(0);
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[0] = userFunction;
#endif
    EICRA &= (~_BV(0) & ~_BV(1)); // reset the flags prior to
    EICRA |= mode;                // set them the way we want
    EIFR |= _BV(0);
    EIMSK |= _BV(0);
    break;
#endif
#ifndef EI_NOTINT1
  case ARDUINO_PIN_D3 : // INT1  (Either 3 or 11)
    EIMSK &= ~_BV(1);
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[1] = userFunction;
#endif
    EICRA &= (~_BV(2) & ~_BV(3));
    EICRA |= (mode << 2);
    EIFR |= _BV(1);
    EIMSK |= _BV(1);
    break;
#endif
#ifndef EI_NOTINT2
  case ARDUINO_PIN_B2 : // INT2  (Either 10 or 2)
    EIMSK &= ~_BV(2);
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[2] = userFunction;
#endif
    EICRA &= (~_BV(4) & ~_BV(5));
    EICRA |= (mode << 4);
    EIFR |= _BV(2);
    EIMSK |= _BV(2);
    break;
#endif
}
#endif // EI_SECTION_ENABLEEXTERNAL

#ifdef EI_SECTION_DISABLEEXTERNAL
#ifndef EI_NOTINT0
if (arduinoPin == ARDUINO_PIN_D2) {  // INT0  (Either 2 or 10)
  EIMSK &= ~_BV(0);
  EICRA &= (~_BV(0) & ~_BV(1));
  EIFR  |= _BV(0); // using a clue from the ATmega2560 datasheet.
}
#endif
#ifndef EI_NOTINT1
if (arduinoPin == ARDUINO_PIN_D3) {  // INT1  (Either 3 or 11)
  EIMSK &= ~_BV(1);
  EICRA &= (~_BV(2) & ~_BV(3));
  EIFR  |= _BV(1); // using a clue from the ATmega2560 datasheet.
}
#endif
#ifndef EI_NOTINT2
if (arduinoPin == ARDUINO_PIN_B2) {  // INT2  (Either 10 or 2)
  EIMSK &= ~_BV(2);
  EICRA &= (~_BV(4) & ~_BV(5));
  EIFR  |= _BV(2); // using a clue from the ATmega2560 datasheet.
}
#endif
#endif // EI_SECTION_DISABLEEXTERNAL
#endif // ! defined(EI_NOTINT0) && ! defined (EI_NOTINT1) && ! defined (EI_NOTINT2)
