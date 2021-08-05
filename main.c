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
#pragma config PGL1WAY   = OFF      // Permission Group Lock One Way Configuration bit (Allow multiple reconfigurations)
#pragma config PMDL1WAY  = OFF     // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY   = OFF      // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
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

#define LEDRED RPG12
#define LEDGREEN RPG13
#define LEDYELLOW RPG14

#define SWITCH1 RG11
#define SWITCH2 RF13
#define SWITCH3 RF12

void delay(unsigned int count) {
    while (--count)
        asm("nop");
}

void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL2AUTO), nomips16)) Timer2_IRQ(void) {
    IFS0CLR = _IFS0_T2IF_MASK;  // clear irq flag

//    LATGINV = _LATG_LATG13_MASK;
}

void __attribute__((vector(_TIMER_4_VECTOR), interrupt(IPL7AUTO), nomips16)) Timer4_IRQ(void) {
    IFS0CLR = _IFS0_T4IF_MASK;  // clear irq flag

    LATGINV = _LATG_LATG14_MASK;
}

// UART4 (TX only)
void uart4init(uint32_t baud) {
    // When using the 1:1 PBCLK divisor, the user software should not read/write the peripheral SFRs in the SYSCLK cycle
    // immediately following the instruction that clears the module’s ON bit.
    U4MODE = 0;  // reset mode
    _nop();
    U4STA             = 0;                                                              // reset status
    U4BRG             = 60000000 / (16 * baud) - 1;                                     // APBclock is sysclk/2 = 60MHz
    U4STAbits.UTXISEL = 2;                                                              // irq when queue is empty
    U4STASET          = _U4STA_UTXEN_MASK;                                              // enable TX
    IFS2CLR           = _IFS2_U4TXIF_MASK;                                              // clear any set irq
    IPC16CLR          = _IPC16_U4TXIS_MASK | _IPC16_U4TXIP_MASK;                        // clear prio & subprio
    IPC16SET          = (3 << _IPC16_U4TXIS_POSITION) | (3 << _IPC16_U4TXIP_POSITION);  // prio 3 (must match handler) subprio 3
    U4MODESET         = _U4MODE_ON_MASK;                                                // on
}

static struct Ringbuffer u4_buf;

void __attribute__((vector(_UART4_TX_VECTOR), interrupt(IPL3AUTO), nomips16)) UART4TX_IRQ(void) {
    IFS2CLR = _IFS2_U4TXIF_MASK;  // clear irq flag

    while (!ringbuffer_empty(&u4_buf) && !(U4STAbits.UTXBF)) {
        U4TXREG = get_tail(&u4_buf);
    }

    if (ringbuffer_empty(&u4_buf)) {
        IEC2CLR = _IEC2_U4TXIE_MASK;  // stop these interrupts
    }

    return;
}

size_t u4puts(const char *buf, size_t len) {
    size_t r = ringbuffer_puts(&u4_buf, buf, len);
    // on overflow zap the buffer and leave a marker for the user that data was lost
    if (r < len) {
        ringbuffer_clear(&u4_buf);
        ringbuffer_puts(&u4_buf, "!OVFL!", 6);
    }

    IEC2SET = _IEC2_U4TXIE_MASK;  // start transmission if not already running
    return r;
}

// 0..31 is the tx fifo, 32..63 is the rx
static struct CanMsg can4fifo[64];


int main(void) {
    __builtin_mtc0(16, 0, (__builtin_mfc0(16, 0) | 0x3));  // CP0.K0 enable cached instruction pre-fetch
    CHECONbits.PFMWS = 3;                                  // prefetch 3 waitstates
    INTCONSET        = _INTCON_MVEC_MASK;                  // use vectored interrupts
    PRISSbits.PRI7SS = 1;                                  // priority level 7 uses the shadow registers

    // ANSELGCLR = 7<<12;
    TRISGbits.TRISG12 = 0;
    TRISGbits.TRISG13 = 0;
    TRISGbits.TRISG14 = 0;

    __builtin_enable_interrupts();

    // UART4
    // ANSELACLR = 1<<12;        
    // ANSELDCLR = 1<<3;        
    TRISAbits.TRISA12 = 0;        // output
    TRISDbits.TRISD3  = 1;        // input
    RPA12R            = 0b00010;  // RPA12 <- UART4 TX
    U4RXR             = 0b1101;   // RPD3  -> UART4 RX
    uart4init(115200);

    // C4TX RPB1
    // C4RX RPC2
    ANSELBCLR = 1<<1;
    ANSELCCLR = 1<<2;
    TRISBbits.TRISB1 = 0; // output
    TRISCbits.TRISC2 = 1; // input
    RPB1R = 0b01100;
    C4RXR = 0b0110;
    can4init(CAN_1MBd, can4fifo);

    LATGSET = _LATG_LATG12_MASK;

    // TIMER 2/3
    // Documentation says the IRQ should come out of the slave but this appears not to be true.

    T3CON    = 0;                                                        // Reset T3 (slave)
    T2CON    = 0;                                                        // and T2 (master)
    T2CONSET = _T2CON_T32_MASK;                                          // enable 32 bit mode
    T2CONSET = 5 << _T2CON_TCKPS_POSITION;                               // prescaler:  (120/2)MHz / (1<<5) = 1875 KHz
    PR2      = 1000000 - 1;                                              // 1.875Hz
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
    PR4      = 1875000 - 1;                                              // 1Hz
    TMR4     = 0;                                                        // reset counter
    IFS0CLR  = _IFS0_T4IF_MASK;                                          // clear flag
    IPC4CLR  = _IPC4_T4IS_MASK | _IPC4_T4IP_MASK;                        // clear prio & subprio
    IPC4SET  = (0 << _IPC4_T4IS_POSITION) | (7 << _IPC4_T4IP_POSITION);  // prio 7 (must match handler), subprio 0
    IEC0SET  = _IEC0_T4IE_MASK;                                          // enable irq
    T4CONSET = _T4CON_ON_MASK;                                           // enable timer

    cbprintf(u4puts, "Booted.\n");

    for (int i = 0;; i++) {
        delay(6000000);
        cbprintf(u4puts, "ping %07d\n", i);
        // u4puts("boo", 3);

        struct CanMsg *msg = c4_tx_head();
        if (msg) {
            uint8_t buf[] = {i, i >> 8, i >> 16, i >> 24};
            mkCanMsg(msg, can_header(0x100, 0), 4, buf);
            c4_tx_push();

            LATGINV = _LATG_LATG12_MASK;

        }

        msg = c4_rx_tail();
        if (msg) {
            char   buf[50];
            can_fmt(buf, sizeof buf, canMsgHeader(msg), canMsgLen(msg), msg->data);
            c4_rx_pull();
            cbprintf(u4puts, "%s\n", buf);

            LATGINV = _LATG_LATG13_MASK;            
        }
    }

    return 0;
}
