#include "can.h"
static inline uintptr_t KVA_TO_PA(void *v) { return (uintptr_t)v & 0x1fffffffUL; }

void can1init(enum CANBaudRate bps, struct CanMsg *fifos) {

    C1CONSET = _C1CON_ON_MASK;  // switch on the module
    
    C1CONbits.REQOP = 0b100;  // config mode
    while (C1CONbits.OPMOD != 0b100)
        _nop();

    C1INTCLR = 0xffff0000; // clear all irq masks

    C1CFGbits.SEG2PHTS = 1;    // freely programmable
    C1CFGbits.SAM      = 1;    // sample 1x/3x
    C1CFGbits.SEG2PH   = 4;    // 5Tq
    C1CFGbits.SEG1PH   = 4;    // 5Tq
    C1CFGbits.PRSEG    = 3;    // 4Tq
    C1CFGbits.SJW      = 1;    // sync adjust up to 2 quanta
    C1CFGbits.BRP      = bps;  // 1+4+5+5 = 15 Tq per bit, so for 1Mbps we want 60/(1+brp) = 15 MHz

    C1FIFOBA = KVA_TO_PA(fifos);

    C1FIFOCON0bits.FSIZE = 31;
    C1FIFOCON0bits.TXEN  = 1;  // fifo 0 is tx
    C1FIFOINT0CLR = 0xffff0000;

    C1FIFOCON1bits.FSIZE = 31;
    C1FIFOCON1bits.TXEN  = 0;  // fifo 1 is rx
    C1FIFOINT1CLR = 0xffff0000;

    C1RXM0              = 0;  // clear mask 0: everything matches
    C1RXF0              = 0;  // filter is don't care
    C1FLTCON0bits.MSEL0 = 0;  // filter 0 use mask 0
    C1FLTCON0bits.FSEL0 = 1;  // filter 0 receive in fifo 1
    C1FLTCON0SET        = _C1FLTCON0_FLTEN0_MASK;

    C1CONbits.REQOP = 0; // normal mode
    while (C1CONbits.OPMOD != 0)
        _nop();
}

void can2init(enum CANBaudRate bps, struct CanMsg *fifos) {

    C2CONSET = _C2CON_ON_MASK;  // switch on the module
    
    C2CONbits.REQOP = 0b100;  // config mode
    while (C2CONbits.OPMOD != 0b100)
        _nop();

    C2INTCLR = 0xffff0000; // clear all irq masks

    C2CFGbits.SEG2PHTS = 1;    // freely programmable
    C2CFGbits.SAM      = 1;    // sample 1x/3x
    C2CFGbits.SEG2PH   = 4;    // 5Tq
    C2CFGbits.SEG1PH   = 4;    // 5Tq
    C2CFGbits.PRSEG    = 3;    // 4Tq
    C2CFGbits.SJW      = 1;    // sync adjust up to 2 quanta
    C2CFGbits.BRP      = bps;  // 1+4+5+5 = 15 Tq per bit, so for 1Mbps we want 60/(1+brp) = 15 MHz

    C2FIFOBA = KVA_TO_PA(fifos);

    C2FIFOCON0bits.FSIZE = 31;
    C2FIFOCON0bits.TXEN  = 1;  // fifo 0 is tx
    C2FIFOINT0CLR = 0xffff0000;

    C2FIFOCON1bits.FSIZE = 31;
    C2FIFOCON1bits.TXEN  = 0;  // fifo 1 is rx
    C2FIFOINT1CLR = 0xffff0000;

    C2RXM0              = 0;  // clear mask 0: everything matches
    C2RXF0              = 0;  // filter is don't care
    C2FLTCON0bits.MSEL0 = 0;  // filter 0 use mask 0
    C2FLTCON0bits.FSEL0 = 1;  // filter 0 receive in fifo 1
    C2FLTCON0SET        = _C2FLTCON0_FLTEN0_MASK;

    C2CONbits.REQOP = 0; // normal mode
    while (C2CONbits.OPMOD != 0)
        _nop();
}


#if 0   // invalid on dev board
void can3init(enum CANBaudRate bps, struct CanMsg *fifos) {
    C3CONSET = _C3CON_ON_MASK;  // switch on the module
    
    C3CONbits.REQOP = 0b100;  // config mode
    while (C3CONbits.OPMOD != 0b100)
        _nop();

    C3INTCLR = 0xffff0000; // clear all irq masks

    C3CFGbits.SEG2PHTS = 1;    // freely programmable
    C3CFGbits.SAM      = 1;    // sample 1x/3x
    C3CFGbits.SEG2PH   = 4;    // 5Tq
    C3CFGbits.SEG1PH   = 4;    // 5Tq
    C3CFGbits.PRSEG    = 3;    // 4Tq
    C3CFGbits.SJW      = 1;    // sync adjust up to 2 quanta
    C3CFGbits.BRP      = bps;  // 1+4+5+5 = 15 Tq per bit, so for 1Mbps we want 60/(1+brp) = 15 MHz

    C3FIFOBA = KVA_TO_PA(fifos);

    C3FIFOCON0bits.FSIZE = 31;
    C3FIFOCON0bits.TXEN  = 1;  // fifo 0 is tx
    C3FIFOINT0CLR = 0xffff0000;

    C3FIFOCON1bits.FSIZE = 31;
    C3FIFOCON1bits.TXEN  = 0;  // fifo 1 is rx
    C3FIFOINT1CLR = 0xffff0000;

    C3RXM0              = 0;  // clear mask 0: everything matches
    C3RXF0              = 0;  // filter is don't care
    C3FLTCON0bits.MSEL0 = 0;  // filter 0 use mask 0
    C3FLTCON0bits.FSEL0 = 1;  // filter 0 receive in fifo 1
    C3FLTCON0SET        = _C3FLTCON0_FLTEN0_MASK;

    C3CONbits.REQOP = 0; // normal mode  
    while (C3CONbits.OPMOD != 0)
        _nop();
}
#endif

void can4init(enum CANBaudRate bps, struct CanMsg *fifos) {

    C4CONSET = _C4CON_ON_MASK;  // switch on the module
    
    C4CONbits.REQOP = 0b100;  // config mode
    while (C4CONbits.OPMOD != 0b100)
        _nop();

    C4INTCLR = 0xffff0000; // clear all irq masks

    C4CFGbits.SEG2PHTS = 1;    // freely programmable
    C4CFGbits.SAM      = 1;    // sample 1x/3x
    C4CFGbits.SEG2PH   = 4;    // 5Tq
    C4CFGbits.SEG1PH   = 4;    // 5Tq
    C4CFGbits.PRSEG    = 3;    // 4Tq
    C4CFGbits.SJW      = 1;    // sync adjust up to 2 quanta
    C4CFGbits.BRP      = bps;  // 1+4+5+5 = 15 Tq per bit, so for 1Mbps we want 60/(1+brp) = 15 MHz

    C4FIFOBA = KVA_TO_PA(fifos);

    C4FIFOCON0bits.FSIZE = 31;
    C4FIFOCON0bits.TXEN  = 1;  // fifo 0 is tx
    C4FIFOINT0CLR = 0xffff0000;

    C4FIFOCON1bits.FSIZE = 31;
    C4FIFOCON1bits.TXEN  = 0;  // fifo 1 is rx
    C4FIFOINT1CLR = 0xffff0000;

    C4RXM0              = 0;  // clear mask 0: everything matches
    C4RXF0              = 0;  // filter is don't care
    C4FLTCON0bits.MSEL0 = 0;  // filter 0 use mask 0
    C4FLTCON0bits.FSEL0 = 1;  // filter 0 receive in fifo 1
    C4FLTCON0SET        = _C4FLTCON0_FLTEN0_MASK;

    C4CONbits.REQOP = 0; // normal mode
    while (C4CONbits.OPMOD != 0)
        _nop();
}
