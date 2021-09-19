/*
 * File:   main.c
 * Author: lvd
 *
 * Created on July 17, 2021, 9:46 PM
 */

// PIC32MK1024GPE100 Configuration Bit Settings

// DEVCFG3
#pragma config USERID    = 0xFFFF  // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config FUSBIDIO2 = ON      // USB2 USBID Selection (USBID pin is controlled by the USB2 module)
#pragma config FVBUSIO2  = ON      // USB2 VBUSON Selection bit (VBUSON pin is controlled by the USB2 module)
#pragma config PGL1WAY   = OFF     // Permission Group Lock One Way Configuration bit (Allow multiple reconfigurations)
#pragma config PMDL1WAY  = OFF     // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY   = OFF     // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO1 = ON      // USB1 USBID Selection (USBID pin is controlled by the USB1 module)
#pragma config FVBUSIO1  = ON      // USB2 VBUSON Selection bit (VBUSON pin is controlled by the USB1 module)

// DEVCFG2
#pragma config FPLLIDIV  = DIV_2           // System PLL Input Divider (2x Divider)
#pragma config FPLLRNG   = RANGE_5_10_MHZ  // System PLL Input Range (5-10 MHz Input)
#pragma config FPLLICLK  = PLL_POSC        // System PLL Input Clock Selection (POSC is input to the System PLL)
#pragma config FPLLMULT  = MUL_40          // System PLL Multiplier (PLL Multiply by 40)
#pragma config FPLLODIV  = DIV_2           // System PLL Output Clock Divider (2x Divider)
#pragma config VBATBOREN = ON              // VBAT BOR Enable (Enable ZPBOR during VBAT Mode)
#pragma config DSBOREN   = ON              // Deep Sleep BOR Enable (Enable ZPBOR during Deep Sleep Mode)
#pragma config DSWDTPS   = DSPS32          // Deep Sleep Watchdog Timer Postscaler (1:2^36)
#pragma config DSWDTOSC  = LPRC            // Deep Sleep WDT Reference Clock Selection (Select LPRC as DSWDT Reference clock)
#pragma config DSWDTEN   = ON              // Deep Sleep Watchdog Timer Enable (Enable DSWDT during Deep Sleep Mode)
#pragma config FDSEN     = ON              // Deep Sleep Enable (Enable DSEN bit in DSCON)
#pragma config BORSEL    = HIGH            // Brown-out trip voltage (BOR trip voltage 2.1v (Non-OPAMP deviced operation))
#pragma config UPLLEN    = OFF             // USB PLL Enable (USB PLL Disabled)

// DEVCFG1
#pragma config FNOSC     = SPLL         // Oscillator Selection Bits (System PLL)
#pragma config DMTINTV   = WIN_127_128  // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN   = ON           // Secondary Oscillator Enable (Enable Secondary Oscillator)
#pragma config IESO      = ON           // Internal/External Switch Over (Enabled)
#pragma config POSCMOD   = HS           // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC  = OFF          // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM     = CSECME       // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS     = PS1048576    // Watchdog Timer Postscaler (1:1048576)
#pragma config WDTSPGM   = STOP         // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS    = NORMAL       // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN    = OFF          // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT    = DMT31        // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN    = OFF          // Deadman Timer Enable (Deadman Timer is enabled)

// DEVCFG0
#pragma config DEBUG     = OFF           // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN    = ON            // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL    = ICS_PGx1      // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN     = ON            // Trace Enable (Trace features in the CPU are enabled)
#pragma config BOOTISA   = MIPS32        // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FSLEEP    = OFF           // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER    = PG_ALL        // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR     = MCLR_NORM     // Soft Master Clear Enable (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN  = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON            // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN  = GAIN_LEVEL_3  // Primary Oscillator Gain Control bits (Gain Level 3 (highest))
#pragma config POSCBOOST = ON            // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN  = NORMAL        // EJTAG Boot Enable (Normal EJTAG functionality)

// DEVCP
#pragma config CP = OFF  // Code Protect (Protection Disabled)

// SEQ
#pragma config TSEQ = 0xFFFF  // Boot Flash True Sequence Number (Enter Hexadecimal value)
#pragma config CSEQ = 0xFFFF  // Boot Flash Complement Sequence Number (Enter Hexadecimal value)

#include <xc.h>

#include "can.h"
#include "fmtcan.h"
#include "printf.h"

#include "PIC32GPDEVBOARD.h"
extern inline void initIOPins(void);  // force compiler to emit definition here

// DS60001145U PIC32 Flash Programming Specification p.69 
// Table C11 PIC32MK GENERAL PURPOSE AND MOTOR CONTROL (GP/MC) FAMILY DEVICE IDS
static const struct devid2str_t {
    uint16_t id; // devid >> 12
    const char* str; 
} PIC32MKdevid2str[] = {
  /*0x16201053*/ { 0x6201, "1024MCF100" },
  /*0x16202053*/ { 0x6202, "1024MCF064" },
  /*0x16204053*/ { 0x6204, "0512MCF100" },
  /*0x16205053*/ { 0x6205, "0512MCF064" },
  /*0x16207053*/ { 0x6207, "1024GPE100" },
  /*0x16208053*/ { 0x6208, "1024GPE064" },
  /*0x1620A053*/ { 0x620A, "0512GPE100" },
  /*0x1620B053*/ { 0x620B, "0512GPE064" },
  /*0x1620D053*/ { 0x620D, "1024GPD100" },
  /*0x1620E053*/ { 0x620E, "1024GPD064" },
  /*0x16210053*/ { 0x6210, "0512GPD100" },
  /*0x16211053*/ { 0x6211, "0512GPD064" },
  /*0x08B01053*/ { 0x8B01, "1024MCM100" },
  /*0x08B02053*/ { 0x8B02, "1024MCM064" },
  /*0x08B04053*/ { 0x8B04, "0512MCM100" },
  /*0x08B05053*/ { 0x8B05, "0512MCM064" },
  /*0x08B07053*/ { 0x8B07, "1024GPL100" },
  /*0x08B08053*/ { 0x8B08, "1024GPL064" },
  /*0x08B0A053*/ { 0x8B0A, "0512GPL100" },
  /*0x08B0B053*/ { 0x8B0B, "0512GPL064" },
  /*0x08B0D053*/ { 0x8B0D, "1024GPK100" },
  /*0x08B0E053*/ { 0x8B0E, "1024GPK064" },
  /*0x08B10053*/ { 0x8B10, "0512GPK100" },
  /*0x08B11053*/ { 0x8B11, "0512GPK064" },
  /*0x06300053*/ { 0x6300, "0512MCH064" },
  /*0x06301053*/ { 0x6301, "0512MCH048" },
  /*0x06302053*/ { 0x6302, "0512MCH040" },
  /*0x06304053*/ { 0x6304, "0256MCH064" },
  /*0x06305053*/ { 0x6305, "0256MCH048" },
  /*0x06306053*/ { 0x6306, "0256MCH040" },
  /*0x06308053*/ { 0x6308, "0512GPG064" },
  /*0x06309053*/ { 0x6309, "0512GPG048" },
  /*0x0630A053*/ { 0x630A, "0512GPG040" },
  /*0x0630C053*/ { 0x630C, "0256GPG064" },
  /*0x0630D053*/ { 0x630D, "0256GPG048" },
  /*0x0630E053*/ { 0x630E, "0256GPG040" },
    {0, 0},
};




void delay(unsigned int count) {
    while (--count)
        asm("nop");
}

void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL2AUTO), nomips16)) Timer2_IRQ(void) {
    IFS0CLR = _IFS0_T2IF_MASK;  // clear irq flag

    //...
}

void __attribute__((vector(_TIMER_4_VECTOR), interrupt(IPL7AUTO), nomips16)) Timer4_IRQ(void) {
    IFS0CLR = _IFS0_T4IF_MASK;  // clear irq flag

    LATGINV = G_LEDYLW;
}

// UART6 (TX only)
void uart6init(uint32_t baud) {
    U6MODE            = 0;                                                              // reset mode
    U6STA             = 0;                                                              // reset status
    U6BRG             = 60000000 / (16 * baud) - 1;                                     // APBclock is sysclk/2 = 60MHz
    U6STAbits.UTXISEL = 2;                                                              // irq when queue is empty
    U6STASET          = _U6STA_UTXEN_MASK;                                              // enable TX
    IFS5CLR           = _IFS5_U6TXIF_MASK;                                              // clear any set irq
    IPC41CLR          = _IPC41_U6TXIS_MASK | _IPC41_U6TXIP_MASK;                        // clear prio & subprio
    IPC41SET          = (3 << _IPC41_U6TXIS_POSITION) | (3 << _IPC41_U6TXIP_POSITION);  // prio 3 (must match handler) subprio 3
    U6MODESET         = _U6MODE_ON_MASK;                                                // on
}

static struct Ringbuffer u6_buf;

void __attribute__((vector(_UART6_TX_VECTOR), interrupt(IPL3AUTO), nomips16)) UART6TX_IRQ(void) {
    IFS5CLR = _IFS5_U6TXIF_MASK;  // clear irq flag

    while (!ringbuffer_empty(&u6_buf) && !(U6STAbits.UTXBF)) {
        U6TXREG = get_tail(&u6_buf);
    }

    if (ringbuffer_empty(&u6_buf)) {
        IEC5CLR = _IEC5_U6TXIE_MASK;  // stop these interrupts
    }

    return;
}

size_t u6puts(const char *buf, size_t len) {
    size_t r = ringbuffer_puts(&u6_buf, buf, len);
    // on overflow zap the buffer and leave a marker for the user that data was lost
    if (r < len) {
        ringbuffer_clear(&u6_buf);
        ringbuffer_puts(&u6_buf, "!OVFL!", 6);
    }

    IEC5SET = _IEC5_U6TXIE_MASK;  // start transmission if not already running
    return r;
}

// 0..31 is the tx fifo, 32..63 is the rx
static struct CanMsg can1fifo[64];
static struct CanMsg can2fifo[64];

int main(void) {
    __builtin_mtc0(16, 0, (__builtin_mfc0(16, 0) | 0x3));  // CP0.K0 enable cached instruction pre-fetch
    CHECONbits.PFMWS = 3;                                  // prefetch 3 waitstates
    INTCONSET        = _INTCON_MVEC_MASK;                  // use vectored interrupts
    PRISSbits.PRI7SS = 1;                                  // priority level 7 uses the shadow registers

    initIOPins();

    __builtin_enable_interrupts();

    uart6init(115200);

    can1init(CAN_1MBd, can1fifo);
    can2init(CAN_1MBd, can2fifo);

    LATGSET = G_LEDRED;

    // TIMER 2/3
    // Documentation says the IRQ should come out of the slave but this appears not to be true.

    T3CON    = 0;                                                        // Reset T3 (slave)
    T2CON    = 0;                                                        // and T2 (master)
    T2CONSET = _T2CON_T32_MASK;                                          // enable 32 bit mode
    T2CONSET = 5 << _T2CON_TCKPS_POSITION;                               // prescaler:  (120/2)MHz / (1<<5) = 1875 KHz
    PR2      = 1875000 - 1;                                              // 1Hz
    TMR2     = 0;                                                        // reset counter
    IFS0CLR  = _IFS0_T2IF_MASK;                                          // clear flag.
    IPC2CLR  = _IPC2_T2IS_MASK | _IPC2_T2IP_MASK;                        // clear prio & subprio
    IPC2SET  = (3 << _IPC2_T2IS_POSITION) | (2 << _IPC2_T2IP_POSITION);  // prio 2 (must match handler), subprio 3
    IEC0SET  = _IEC0_T2IE_MASK;                                          // enable irq
    T2CONSET = _T2CON_ON_MASK;                                           // enable timer

    // TIMER 4/5
    T5CON    = 0;                                                        // Reset T5 (slave)
    T4CON    = 0;                                                        // and T4 (master)
    T4CONSET = _T4CON_T32_MASK;                                          // enable 32 bit mode
    T4CONSET = 5 << _T4CON_TCKPS_POSITION;                               // prescaler: (120/2)MHz / (1<<5) = 1875 KHz
    PR4      = 1000000 - 1;                                              // 1.875Hz
    TMR4     = 0;                                                        // reset counter
    IFS0CLR  = _IFS0_T4IF_MASK;                                          // clear flag
    IPC4CLR  = _IPC4_T4IS_MASK | _IPC4_T4IP_MASK;                        // clear prio & subprio
    IPC4SET  = (0 << _IPC4_T4IS_POSITION) | (7 << _IPC4_T4IP_POSITION);  // prio 7 (must match handler), subprio 0
    IEC0SET  = _IEC0_T4IE_MASK;                                          // enable irq
    T4CONSET = _T4CON_ON_MASK;                                           // enable timer

    for (const struct devid2str_t* p = PIC32MKdevid2str; p->id; ++p) {
        if ( ((DEVID >> 12)&0xffff) == p->id) {
           cbprintf(u6puts, "PIC32MK%s ", p->str);
        }
    }
    cbprintf(u6puts, "(0x%x) v%d\n", (DEVID & _DEVID_DEVID_MASK) >> _DEVID_DEVID_POSITION, (DEVID & _DEVID_VER_MASK) >> _DEVID_VER_POSITION);
    cbprintf(u6puts, "SERIAL %08x:%08x:%08x:%08x\n", DEVSN0,DEVSN1,DEVSN2,DEVSN3);


    T7CONbits.T32   = 1;  // 32 bit mode enabled.
    T7CONbits.TCKPS = 1;  // PB5 clock with prescaler of 1
    T7CONbits.ON    = 1;

    for (int i = 0;; i++) {
        delay(6000000);

        cbprintf(u6puts, "ping %07d %d\n", i, TMR7);
        // u4puts("boo", 3);

        struct CanMsg *msg = c1_tx_head();
        if (msg) {
            uint8_t buf[] = {i, i >> 8, i >> 16, i >> 24};
            mkCanMsg(msg, can_header(0x100, 0), 4, buf);
            c1_tx_push();

            LATGINV = G_LEDRED;
        }

        for (;;) {
            msg = c2_rx_tail();
            if (!msg)
                break;

            char buf[50];
            can_fmt(buf, sizeof buf, canMsgHeader(msg), canMsgLen(msg), msg->data);
            c2_rx_pull();
            cbprintf(u6puts, "%s\n", buf);

            LATGINV = G_LEDGRN;
        }
    }

    return 0;
}

//  Exception handler

static size_t exception_puts(const char *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        while (!U6STAbits.TRMT) {
            _nop();
        }
        U6TXREG = buf[i];
    }
    return len;
}

static const char *exc_str[] = {
        "interrupt",  // IRQ = 0,
        "unknown1",
        "unknown2",
        "unknown3",
        "address error exception (load or ifetch)",  //     AdEL = 4,
        "address error exception (store)",           //   AdES,
        "bus error (ifetch)",                        //   IBE,
        "bus error (load/store)",                    //     DBE,
        "syscall",                                   //     Sys,
        "breakpoint",                                //    Bp,
        "reserved instruction",                      //      RI,
        "coprocessor unusable",                      //   CpU,
        "arithmetic overflow",                       //   Overflow,
        "trap (possible divide by zero)",            //     Trap,
        "unknown14",
        "unknown15",
        "implementation specific 1",  //   IS1 = 16,
        "CorExtend Unuseable",        //   CEU,
        "coprocessor 2",              //    C2E
};

// this function overrides the normal _weak_ generic handler
void __attribute__((noreturn)) _general_exception_handler(void) {
    uint32_t code;
    uint32_t addr;

    asm volatile("mfc0 %0,$13" : "=r"(code));
    asm volatile("mfc0 %0,$14" : "=r"(addr));

    code &= 0xff;
    code >>= 2;
    const char* s = "UNKNOWN";
    if (code < sizeof exc_str) {
        s = exc_str[code];
    }

    cbprintf(exception_puts, "EXCEPTION %x %s @%x\n\n", code, s, addr);

#if 0
    SYSKEY = 0x00000000; //write invalid key to force lock
    SYSKEY = 0xAA996655; //write key1 to SYSKEY
    SYSKEY = 0x556699AA; //write key2 to SYSKEY
    // OSCCON is now unlocked
    /* set SWRST bit to arm reset */
    RSWRSTSET = 1;
    /* read RSWRST register to trigger reset */
    _exception_code = RSWRST;
#endif

    for (;;) {}
}




