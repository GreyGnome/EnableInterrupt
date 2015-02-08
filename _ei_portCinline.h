#define EI_ASM_PREFIX \
  /* BEGASM This: \
  current = PINC; // PortC Input. \
  is the same as this: */ \
  asm volatile("\t" \
  "push %0" "\t\n\t" \
  "in %0,%1" "\t\n\t" \
  : "=&r" (current) \
  : "I" (_SFR_IO_ADDR(PINC)) \
  ); \
  /* ENDASM ...End of the sameness.*/ \
 \
   asm volatile( \
  "push r1" "\n\t" \
  "push r0" "\n\t" \
  /* in 0x3f saves SREG... then it's pushed onto the stack.*/ \
  "in r0, __SREG__" "\n\t" /* 0x3f */\
  "push r0" "\n\t" \
  "eor r1, r1" "\n\t" \
  "push r18" "\n\t" \
  "push r19" "\n\t" \
  "push r20" "\n\t" \
  "push r21" "\n\t" \
  "push r22" "\n\t" \
  "push r23" "\n\t" \
  "push r24" "\n\t" \
  "push r26" "\n\t" \
  "push r27" "\n\t" \
  "push r28" "\n\t" \
  "push r29" "\n\t" \
  "push r30" "\n\t" \
  "push r31" "\n\t" \
  : \
  :)

#define EI_ASM_SUFFIX \
 asm volatile( \
  "pop r31" "\n\t" \
  "pop r30" "\n\t" \
  "pop r29" "\n\t" \
  "pop r28" "\n\t" \
  "pop r27" "\n\t" \
  "pop r26" "\n\t" \
  "pop r24" "\n\t" \
  "pop r23" "\n\t" \
  "pop r22" "\n\t" \
  "pop r21" "\n\t" \
  "pop r20" "\n\t" \
  "pop r19" "\n\t" \
  "pop r18" "\n\t" \
  "pop r0" "\n\t" \
  "out __SREG__, r0" "\t\n\t" \
  "pop r0" "\t\n\t" \
  "pop r1" "\t\n\t" \
  "pop r25" "\n\t" \
  "reti" "\t\n\t" \
  : \
  :)

//inlineISR(PC, portSnapshotC, risingPinsPORTC, fallingPinsPORTC, PCMSK1, functionPointerArrayPORTC);
//__attribute__((always_inline)) void inlineISR(uint8_t current,
inline void inlineISR(uint8_t current,
    volatile uint8_t *portSnapshot,
    uint8_t risingPins, uint8_t fallingPins,
    volatile uint8_t pcmsk,
    interruptFunctionType functionPointerArray[]) {
  uint8_t i;
  uint8_t interruptMask;
  uint8_t changedPins;

  changedPins=(*portSnapshot ^ current) & ((risingPins & current) | (fallingPins & ~current));
  *portSnapshot =  current;
  if (changedPins == 0) goto exitISR; // get out quickly if not interested.

  interruptMask = pcmsk & changedPins;
  if (interruptMask == 0) goto exitISR;
  i=0;
  while (1) {
    if (interruptMask & 0x01) {
      (*functionPointerArray[i])();
    }
    interruptMask=interruptMask >> 1;
    if (interruptMask == 0) goto exitISR;
    i++;
  }
  exitISR: return;
}

/*
ISR(PORTC_VECT, ISR_NAKED) {
  uint8_t current;

  EI_ASM_PREFIX;

  inlineISR(current,
      &portSnapshotC,
      risingPinsPORTC,
      fallingPinsPORTC,
      PCMSK1,
      functionPointerArrayPORTC );

  EI_ASM_SUFFIX;
}
*/
volatile uint8_t pinstate;
ISR(PORTC_VECT) {
  pinstate=PINC;
}
