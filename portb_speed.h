#if defined LEONARDO
#ifdef INTERRUPT_FLAG_PINSS
  if (interruptMask & _BV(0)) INTERRUPT_FLAG_PINSS++;
#endif
#ifdef INTERRUPT_FLAG_PINSCK
  if (interruptMask & _BV(1)) INTERRUPT_FLAG_PINSCK++;
#endif
#ifdef INTERRUPT_FLAG_PINMOSI
  if (interruptMask & _BV(2)) INTERRUPT_FLAG_PINMOSI++;
#endif
#ifdef INTERRUPT_FLAG_PINMISO
  if (interruptMask & _BV(3)) INTERRUPT_FLAG_PINMISO++;
#endif
#ifdef INTERRUPT_FLAG_PIN8
  if (interruptMask & _BV(4)) INTERRUPT_FLAG_PIN8++;
#endif
#ifdef INTERRUPT_FLAG_PIN9
  if (interruptMask & _BV(5)) INTERRUPT_FLAG_PIN9++;
#endif
#ifdef INTERRUPT_FLAG_PIN10
  if (interruptMask & _BV(6)) INTERRUPT_FLAG_PIN10++;
#endif
#ifdef INTERRUPT_FLAG_PIN11
  if (interruptMask & _BV(7)) INTERRUPT_FLAG_PIN11++;
#endif
#endif

#if defined ARDUINO_328
#ifdef INTERRUPT_FLAG_PIN8
  if (interruptMask & _BV(0)) INTERRUPT_FLAG_PIN8++;
#endif
#ifdef INTERRUPT_FLAG_PIN9
  if (interruptMask & _BV(1)) INTERRUPT_FLAG_PIN9++;
#endif
#ifdef INTERRUPT_FLAG_PIN10
  if (interruptMask & _BV(2)) INTERRUPT_FLAG_PIN10++;
#endif
#ifdef INTERRUPT_FLAG_PIN11
  if (interruptMask & _BV(3)) INTERRUPT_FLAG_PIN11++;
#endif
#ifdef INTERRUPT_FLAG_PIN12
  if (interruptMask & _BV(4)) INTERRUPT_FLAG_PIN12++;
#endif
#ifdef INTERRUPT_FLAG_PIN13
  if (interruptMask & _BV(5)) INTERRUPT_FLAG_PIN13++;
#endif
#endif

#if defined ARDUINO_MEGA
#ifdef INTERRUPT_FLAG_PINSS
  if (interruptMask & _BV(0)) INTERRUPT_FLAG_PINSS++;
#endif
#ifdef INTERRUPT_FLAG_PINSCK
  if (interruptMask & _BV(1)) INTERRUPT_FLAG_PINSCK++;
#endif
#ifdef INTERRUPT_FLAG_PINMOSI
  if (interruptMask & _BV(2)) INTERRUPT_FLAG_PINMOSI++;
#endif
#ifdef INTERRUPT_FLAG_PINMISO
  if (interruptMask & _BV(3)) INTERRUPT_FLAG_PINMISO++;
#endif
#ifdef INTERRUPT_FLAG_PIN10
  if (interruptMask & _BV(4)) INTERRUPT_FLAG_PIN10++;
#endif
#ifdef INTERRUPT_FLAG_PIN11
  if (interruptMask & _BV(5)) INTERRUPT_FLAG_PIN11++;
#endif
#ifdef INTERRUPT_FLAG_PIN12
  if (interruptMask & _BV(6)) INTERRUPT_FLAG_PIN12++;
#endif
#ifdef INTERRUPT_FLAG_PIN13
  if (interruptMask & _BV(7)) INTERRUPT_FLAG_PIN13++;
#endif
#endif
