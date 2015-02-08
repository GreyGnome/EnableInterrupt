#define EI_PORTLETTER C
#define EI_PCMSK PCMSK1

#undef EI_VECTOR
#undef EI_PIN
#undef EI_SNAPSHOT
#undef EI_RISING
#undef EI_FALLING
#undef EI_FUNCTIONARRAY

#define EI_VECT_(x) PORT ##x ##_VECT
#define EI_P_(x) PIN ##x
#define EI_PS_(x) portSnapshot ##x
#define EI_RPP_(x) risingPinsPORT ##x
#define EI_FPP_(x) fallingPinsPORT ##x
#define EI_FPAP_(x) functionPointerArrayPORT ##x

#define EI_VECT_IN(x) EI_VECT_(x)
#define EI_P_IN(x) EI_P_(x)
#define EI_PS_IN(x) EI_PS_(x)
#define EI_RPP_IN(x) EI_RPP_(x)
#define EI_FPP_IN(x) EI_FPP_(x)
#define EI_FPAP_IN(x) EI_FPAP_(x)

#define EI_VECTOR         EI_VECT_IN(EI_PORTLETTER)
#define EI_PIN            EI_P_IN(EI_PORTLETTER)
#define EI_PORTSNAPSHOT   EI_PS_IN(EI_PORTLETTER)
#define EI_RISING         EI_RPP_IN(EI_PORTLETTER)
#define EI_FALLING        EI_FPP_IN(EI_PORTLETTER)
#define EI_FUNCTIONARRAY  EI_FPAP_IN(EI_PORTLETTER)

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
ISR(EI_VECTOR, ISR_NAKED) {
//ISR(EI_VECTOR) {
  uint8_t current;
  uint8_t i;
  uint8_t interruptMask;
  uint8_t changedPins;

  // BEGASM This: 
	// current = PINC; // PortC Input.
  // is the same as this:
  asm volatile("\t"
  "push %0" "\t\n\t"
  "in %0,%1" "\t\n\t"
  : "=&r" (current)
  : "I" (_SFR_IO_ADDR(EI_PIN))
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
  "push r25" "\n\t"
  "push r26" "\n\t"
  "push r27" "\n\t"
  "push r28" "\n\t"
  "push r29" "\n\t"
  "push r30" "\n\t"
  "push r31" "\n\t"
  :
  :); 

  //interruptSaysHello(); // DEBUG
  //interruptsCalled++; // DEBUG
  changedPins=(EI_PORTSNAPSHOT ^ current) & ((EI_RISING & current) | (EI_FALLING & ~current));
  EI_PORTSNAPSHOT = current;
  interruptMask = EI_PCMSK & changedPins; // We are only interested in pins that were changed
                                        // AND are marked in PCMSK1.
  if (interruptMask == 0) goto exitISR; // Get out quickly if nothing we are interested in changed.
  i=0;
  while (1) {
    if (interruptMask & 0x01) {
      //interruptSaysHello(); // DEBUG
      (*EI_FUNCTIONARRAY[i])();
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
  "pop r28" "\n\t"
  "pop r27" "\n\t"
  "pop r26" "\n\t"
  "pop r25" "\n\t"
  "pop r23" "\n\t"
  "pop r22" "\n\t"
  "pop r21" "\n\t"
  "pop r20" "\n\t"
  "pop r19" "\n\t"
  "pop r18" "\n\t"
  "pop r0" "\n\t"
  "out __SREG__, r0" "\t\n\t"
  "pop r0" "\t\n\t"
  "pop r1" "\t\n\t"
  "pop r24" "\n\t"
  "reti" "\t\n\t"
  :
  :);
}


