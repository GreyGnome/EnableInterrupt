#ifdef EI_SECTION_ENABLEEXTERNAL
switch (arduinoPin) {
  case 10 : // INT0
    EIMSK &= ~_BV(0);
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[0] = userFunction;
#endif
    EICRA &= (~_BV(0) & ~_BV(1)); // reset the flags prior to
    EICRA |= mode;                // set them the way we want
    EIFR |= _BV(0);
    EIMSK |= _BV(0);
    break;
  case 11 : // INT1
    EIMSK &= ~_BV(1);
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[1] = userFunction;
#endif
    EICRA &= (~_BV(2) & ~_BV(3));
    EICRA |= (mode << 2);
    EIFR |= _BV(1);
    EIMSK |= _BV(1);
    break;
  case 2 : // INT2
    EIMSK &= ~_BV(2);
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[2] = userFunction;
#endif
    EICRA &= (~_BV(4) & ~_BV(5));
    EICRA |= (mode << 4);
    EIFR |= _BV(2);
    EIMSK |= _BV(2);
    break;
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
