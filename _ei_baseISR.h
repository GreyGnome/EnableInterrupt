#undef _EI_VECTOR
#undef _EI_PIN
#undef _EI_SNAPSHOT
#undef _EI_RISING
#undef _EI_FALLING
#undef _EI_PCMSK
#undef _EI_FUNCTIONARRAY

#if _EI_PORTLETTER == 'B'
  #define _EI_VECTOR PORTB_VECT
  #define _EI_PIN  PINB
  #define _EI_SNAPSHOT portSnapshotB
  #define _EI_RISING risingPinsPORTB
  #define _EI_FALLING fallingPinsPORTB
  #define _EI_PCMSK PCMSK0
  #define _EI_FUNCTIONARRAY functionPointerArrayPORTB
#elif _EI_PORTLETTER == 'C'
  #define _EI_VECTOR PORTC_VECT
  #define _EI_PIN  PINC
  #define _EI_SNAPSHOT portSnapshotC
  #define _EI_RISING risingPinsPORTC
  #define _EI_FALLING fallingPinsPORTC
  #define _EI_PCMSK PCMSK1
  #define _EI_FUNCTIONARRAY functionPointerArrayPORTC
#elif _EI_PORTLETTER == 'D'
  #define _EI_VECTOR PORTD_VECT
  #define _EI_PIN PIND
  #define _EI_SNAPSHOT portSnapshotD
  #define _EI_RISING risingPinsPORTD
  #define _EI_FALLING fallingPinsPORTD
  #define _EI_PCMSK PCMSK2
  #define _EI_FUNCTIONARRAY functionPointerArrayPORTD
#endif

/* NOTA BIEN:
 * *any* change to this ISR requires an investigation via an object dump, and the register
 * used for "current" needs to be noted. That register must be the first one pushed, and the
 * sequence of "push" commands adjusted to remove that register from the main group.
 * Then, at the end of the ISR, that register needs to be popped LAST.
 *
 * All "vector_X" functions in the assembly language output of the objdump should be reviewed.
 * They *should* use an identical register for "current", but woe betide us if they do not
 * for some reason!
 */
ISR(_EI_VECTOR, ISR_NAKED) {
  uint8_t current;
  uint8_t i;
  uint8_t interruptMask;

  // BEGASM This: 
	//current = PINC; // PortC Input.
  // is the same as this:
  asm volatile("\t"
  "push %0" "\t\n\t"
  "in %0,%1" "\t\n\t"
  : "=&r" (current)
  : "I" (_SFR_IO_ADDR(_EI_PIN))
  );
  // ENDASM ...End of the same.

  // Arrgh... dump of asm shows that 'current' == r25 (from the in, above). So we push all other
  // registers. Is there no mechanism to say 'push all registers except X'? I doubt it. So I just
  // have to check it here, after every time we modify this ISR...
  asm volatile(
  "push r1" "\n\t"
  "push r0" "\n\t"
  // in 0x3f saves SREG... then it's pushed onto the stack.
  "in r0, __SREG__" "\n\t" // 0x3f
  "push r0" "\n\t"
  "eor r1, r1" "\n\t"
  "push r18" "\n\t"
  "push r19" "\n\t"
  "push r20" "\n\t"
  "push r21" "\n\t"
  "push r22" "\n\t"
  "push r23" "\n\t"
  "push r24" "\n\t"
  "push r25" "\n\t"
  "push r26" "\n\t"
  "push r27" "\n\t"
  "push r29" "\n\t"
  "push r30" "\n\t"
  "push r31" "\n\t"
  :
  :); 

  interruptSaysHello(); // DEBUG
  //interruptsCalled++; // DEBUG
  changedPins=(_EI_SNAPSHOT ^ current) & ((_EI_RISING & current) | (_EI_RISING & ~current));
  _EI_SNAPSHOT = current;
  interruptMask = _EI_PCMSK & changedPins; // We are only interested in pins that were changed
                                        // AND are marked in PCMSK1.
  if (interruptMask == 0) goto exitISR; // Get out quickly if nothing we are interested in changed.
  i=0;
  while (1) {
    if (interruptMask & 0x01) {
      interruptSaysHello(); // DEBUG
      (*_EI_FUNCTIONARRAY[i])();
    }
    interruptMask=interruptMask >> 1;
    if (interruptMask == 0) goto exitISR;
    i++;
  }
  exitISR:
  asm volatile(
  "pop r31" "\n\t"
  "pop r30" "\n\t"
  "pop r29" "\n\t"
  "pop r27" "\n\t"
  "pop r26" "\n\t"
  "pop r25" "\n\t"
  "pop r24" "\n\t"
  "pop r23" "\n\t"
  "pop r22" "\n\t"
  "pop r20" "\n\t"
  "pop r21" "\n\t"
  "pop r19" "\n\t"
  "pop r18" "\n\t"
  "pop r0" "\n\t"
  "out __SREG__, r0" "\t\n\t"
  "pop r0" "\t\n\t"
  "pop r1" "\t\n\t"
  "pop r28" "\n\t"
  "reti" "\t\n\t"
  :
  :);
}


