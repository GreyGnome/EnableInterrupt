#ifdef EI_SECTION_ENABLEEXTERNAL
if (arduinoPin == 3) {
#ifndef NEEDFORSPEED
  functionPointerArrayEXTERNAL[1] = userFunction;
#endif
  EIMSK &= ~_BV(1);
  EICRA &= (~_BV(2) & ~_BV(3));
  EICRA |= mode << 2;
  EIFR  |= _BV(1); // using a clue from the ATmega2560 datasheet.
  EIMSK |= _BV(1);
} else {
#ifndef NEEDFORSPEED
  functionPointerArrayEXTERNAL[0] = userFunction;
#endif
  EIMSK &= ~_BV(0);
  EICRA &= (~_BV(0) & ~_BV(1));
  EICRA |= mode;
  EIFR  |= _BV(0); // using a clue from the ATmega2560 datasheet.
  EIMSK |= _BV(0);
}
#endif

#ifdef EI_SECTION_DISABLEEXTERNAL
if (arduinoPin == 3) {
  EIMSK &= ~_BV(1);
  EICRA &= (~_BV(2) & ~_BV(3));
  EIFR  |= _BV(1); // using a clue from the ATmega2560 datasheet.
} else {
  EIMSK &= ~_BV(0);
  EICRA &= (~_BV(0) & ~_BV(1));
  EIFR  |= _BV(0); // using a clue from the ATmega2560 datasheet.
}
#endif
