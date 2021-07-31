#include "can.h"

static inline uintptr_t KVA_TO_PA(void *v) { return (uintptr_t)v & 0x1fffffffUL; }

void c3init(enum CANBaudRate bps, struct CanMsg* fifos) {
    LATGSET = _LATG_LATG12_MASK;

    C3CONSET = _IC3CON_ON_MASK;  // switch on thh module

    C3CONbits.REQOP = 0b100;  // config mode
    while (C3CONbits.OPMOD != 0b100)
        _nop();

    LATGSET = _LATG_LATG13_MASK;

    C3CFGbits.SEG2PHTS = 1;    // freely programmable
    C3CFGbits.SAM      = 1;    // sample 3x
    C3CFGbits.SEG2PH   = 4;    // 5Tq
    C3CFGbits.SEG1PH   = 4;    // 5Tq
    C3CFGbits.PRSEG    = 0;    // 1Tq
    C3CFGbits.SJW      = 1;    // sync adjust up to 2 quanta
    C3CFGbits.BRP      = bps;  // 1+1+5+5 = 12 Tq per bit, so for 1Mbps we want 60/(1+brp) = 12 MHz

    C3FIFOBA = KVA_TO_PA(fifos);

    C3FIFOCON0bits.TXEN  = 1;  // fifo 0 is tx
    C3FIFOCON0bits.FSIZE = 31;
    C3FIFOCON1bits.FSIZE = 31;

    C3RXM0              = 0;  // clear mask 0: everything matches
    C3FLTCON0bits.MSEL0 = 0;  // filter 1 use mask 0
    C3RXF0              = 0;  // filter is don't care
    C3FLTCON0bits.FSEL0 = 1;  // filter 1 receive in fifo 1
    C3FLTCON0SET        = _C3FLTCON0_FLTEN0_MASK;

    C3CONbits.REQOP = 0;  // normal mode
    while (C3CONbits.OPMOD != 0)
        _nop();

    LATGSET = _LATG_LATG14_MASK;
}
