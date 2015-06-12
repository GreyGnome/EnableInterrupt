#ifdef EI_SECTION_ENABLEEXTERNAL
switch (arduinoPin) {
  case 21 : // INT0
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[0] = userFunction;
#endif
    EIMSK &= ~_BV(0);
    EICRA &= (~_BV(0) & ~_BV(1));
    EICRA |= mode;
    EIFR |= _BV(0);
    EIMSK |= _BV(0);
    break;
  case 20 : // INT1
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[1] = userFunction;
#endif
    EIMSK &= ~_BV(1);
    EICRA &= (~_BV(2) & ~_BV(3));
    EICRA |= (mode << 2);
    EIFR |= _BV(1);
    EIMSK |= _BV(1);
    break;
  case 19 : // INT2
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[2] = userFunction;
#endif
    EIMSK &= ~_BV(2);
    EICRA &= (~_BV(4) & ~_BV(5));
    EICRA |= (mode << 4);
    EIFR |= _BV(2);
    EIMSK |= _BV(2);
    break;
  case 18 : // INT3
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[3] = userFunction;
#endif
    EIMSK &= ~_BV(3);
    EICRA &= (~_BV(6) & ~_BV(7));
    EICRA |= (mode << 6);
    EIFR |= _BV(3);
    EIMSK |= _BV(3);
    break;
  case  2 : // INT4
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[4] = userFunction;
#endif
    EIMSK &= ~_BV(4);
    EICRB &= (~_BV(0) & ~_BV(1));
    EICRB |= mode;
    EIFR |= _BV(4);
    EIMSK |= _BV(4);
    break;
  case  3 : // INT5
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[5] = userFunction;
#endif
    EIMSK &= ~_BV(5);
    EICRB &= (~_BV(2) & ~_BV(3));
    EICRB |= (mode << 2);
    EIFR |= _BV(5);
    EIMSK |= _BV(5);
    break;
  case 75 : // INT6- Fake Arduino Pin
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[6] = userFunction;
#endif
    EIMSK &= ~_BV(6);
    EICRB &= (~_BV(4) & ~_BV(5));
    EICRB |= (mode << 4);
    EIFR |= _BV(6);
    EIMSK |= _BV(6);
    break;
  case 76 : // INT7- Fake Arduino Pin
#ifndef NEEDFORSPEED
    functionPointerArrayEXTERNAL[7] = userFunction;
#endif
    EIMSK &= ~_BV(7);
    EICRB &= (~_BV(6) & ~_BV(7));
    EICRB |= (mode << 6);
    EIFR |= _BV(7);
    EIMSK |= _BV(7);
    break;
}
#endif

#ifdef EI_SECTION_DISABLEEXTERNAL
switch (arduinoPin) {
  case 21 : // INT0
    EIMSK &= ~_BV(0);
    EICRA &= (~_BV(0) & ~_BV(1));
    EIFR |= _BV(0);
    break;
  case 20 : // INT1
    EIMSK &= ~_BV(1);
    EICRA &= (~_BV(2) & ~_BV(3));
    EIFR |= _BV(1);
    break;
  case 19 : // INT2
    EIMSK &= ~_BV(2);
    EICRA &= (~_BV(4) & ~_BV(5));
    EIFR |= _BV(2);
    break;
  case 18 : // INT3
    EIMSK &= ~_BV(3);
    EICRA &= (~_BV(6) & ~_BV(7));
    EIFR |= _BV(3);
    break; 
  case  2 : // INT4
    EIMSK &= ~_BV(4);
    EICRB &= (~_BV(0) & ~_BV(1));
    EIFR |= _BV(4);
    break;
  case  3 : // INT5
    EIMSK &= ~_BV(5);
    EICRB &= (~_BV(2) & ~_BV(3));
    EIFR |= _BV(5);
    break;
  case 75 : // INT6- Fake Arduino Pin
    EIMSK &= ~_BV(6);
    EICRB &= (~_BV(4) & ~_BV(5));
    EIFR |= _BV(6);
    break;
  case 76 : // INT7- Fake Arduino Pin
    EIMSK &= ~_BV(7);
    EICRB &= (~_BV(6) & ~_BV(7));
    EIFR |= _BV(7);
    break;
}
#endif
