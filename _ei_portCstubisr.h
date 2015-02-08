#error Remove these lines to get this to compile. Then, dump the elf file and study the assembler.
#error DO NOT expect this code to work. It has flawed logic. See the Technical Notes for discussion.
void subISRC(uint8_t current) {
  uint8_t i;
  uint8_t interruptMask;
  uint8_t changedPins;

  changedPins=(portSnapshotC ^ current) & ((risingPinsPORTC & current) | (fallingPinsPORTC & ~current));
  portSnapshotC = current;

  //if (changedPins == 0) return; // get out quickly if not interested.

  interruptMask = PCMSK1 & changedPins;
  if (interruptMask == 0) reti();
  i=0;
  while (1) {
    if (interruptMask & 0x01) {
      (*functionPointerArrayPORTC[i])();
    }
    interruptMask=interruptMask >> 1;
    if (interruptMask == 0) reti();
    i++;
  }
}

ISR(PORTC_VECT, ISR_NAKED) {
  uint8_t current;

  asm volatile("\t"
  "push %0" "\t\n\t"
  "in %0,%1" "\t\n\t"
  : "=&r" (current)
  : "I" (_SFR_IO_ADDR(PINC))
  );

  subISRC(current);
}

