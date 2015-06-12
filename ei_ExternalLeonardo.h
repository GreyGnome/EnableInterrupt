#ifdef EI_SECTION_ENABLEEXTERNAL
switch (arduinoPin) {
  case 3 : // INT0
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[0] = userFunction;
#endif
    EIMSK &= ~_BV(0);
    EICRA &= (~_BV(0) & ~_BV(1));
    EICRA |= mode;
    EIFR |= _BV(0);
    EIMSK |= _BV(0);
    break;
  case 2 : // INT1
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[1] = userFunction;
#endif
    EIMSK &= ~_BV(1);
    EICRA &= (~_BV(2) & ~_BV(3));
    EICRA |= (mode << 2);
    EIFR |= _BV(1);
    EIMSK |= _BV(1);
    break;
  case 0 : // INT2
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[2] = userFunction;
#endif
    EIMSK &= ~_BV(2);
    EICRA &= (~_BV(4) & ~_BV(5));
    EICRA |= (mode << 4);
    EIFR |= _BV(2);
    EIMSK |= _BV(2);
    break;
  case 1 : // INT3
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[3] = userFunction;
#endif
    EIMSK &= ~_BV(3);
    EICRA &= (~_BV(6) & ~_BV(7));
    EICRA |= (mode << 6);
    EIFR |= _BV(3);
    EIMSK |= _BV(3);
    break;
  case 7 : // INT6
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[4] = userFunction;
#endif
    EIMSK &= ~_BV(6);
    EICRB &= (~_BV(4) & ~_BV(5));
    EICRB |= (mode << 4);
    EIFR |= _BV(6);
    EIMSK |= _BV(6);
    break;
}
#endif

#ifdef EI_SECTION_DISABLEEXTERNAL
switch (arduinoPin) {
  case 3 : // INT0
    EIMSK &= ~_BV(0);
    EICRA &= (~_BV(0) & ~_BV(1));
    EIFR |= _BV(0);
    break;
  case 2 : // INT1
    EIMSK &= ~_BV(1);
    EICRA &= (~_BV(2) & ~_BV(3));
    EIFR |= _BV(1);
    break;
  case 0 : // INT2
    EIMSK &= ~_BV(2);
    EICRA &= (~_BV(4) & ~_BV(5));
    EIFR |= _BV(2);
    break;
  case 1 : // INT3
    EIMSK &= ~_BV(3);
    EICRA &= (~_BV(6) & ~_BV(7));
    EIFR |= _BV(3);
    break;
  case 7 : // INT6
    EIMSK &= ~_BV(6);
    EICRB &= (~_BV(4) & ~_BV(5));
    EIFR |= _BV(6);
    break;
}
#endif
