#if defined ARDUINO_328
#if defined INTERRUPT_FLAG_PINA0
  if (interruptMask & _BV(0)) INTERRUPT_FLAG_PINA0++;
#endif
#if defined INTERRUPT_FLAG_PINA1
  if (interruptMask & _BV(1)) INTERRUPT_FLAG_PINA1++;
#endif
#if defined INTERRUPT_FLAG_PINA2
  if (interruptMask & _BV(2)) INTERRUPT_FLAG_PINA2++;
#endif
#if defined INTERRUPT_FLAG_PINA3
  if (interruptMask & _BV(3)) INTERRUPT_FLAG_PINA3++;
#endif
#if defined INTERRUPT_FLAG_PINA4
  if (interruptMask & _BV(4)) INTERRUPT_FLAG_PINA4++;
#endif
#if defined INTERRUPT_FLAG_PINA5
  if (interruptMask & _BV(5)) INTERRUPT_FLAG_PINA5++;
#endif
#endif
