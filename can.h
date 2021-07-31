#pragma once

#include <xc.h>
#include <stdint.h>

/**  uint32 representation of id_a, id_b, the IDE and the RTR bits for in-memory manipulation.

let IDE be the identifier extension flag, when set to 1 meaning the extended id field is present
let RTR be the remote transmission request flag, when set to 1 meaning no payload, but a request

    bit 31 (1 bit): unused
    bit 30-20 (11 bits): ID-A[10:0]               0x7ff_ ____
    bit 19    (1 bit): RTR                        0x___8 ____
    bit 18    (1 bit): IDE                        0x___4 ____
    bit 17-16 (2 bits):  ID-B[17:16]              0x___3 ____
    bit 15-0  (16 bits): ID-B[15:0]               0x____ ffff

*/
inline uint32_t can_header(uint32_t id_a, int32_t rtr) { return ((id_a & 0x7ff) << 20) | ((rtr != 0) ? (1 << 19) : 0); }
inline uint32_t can_header_ext(uint32_t id_a, uint32_t id_b, int32_t rtr) {
	return ((id_a & 0x7ff) << 20) | ((rtr != 0) ? (1 << 19) : 0) | (1 << 18) | (id_b & 0x3ffff);
}
inline int32_t can_header_isext(uint32_t h) { return ((h & (1 << 18)) != 0) ? -1 : 0; }
inline uint32_t can_header_id_a(uint32_t h) { return (h >> 20) & 0x7ff; }
inline uint32_t can_header_id_b(uint32_t h) { return h & 0x3ffff; }
inline uint32_t can_header_setrtr(uint32_t h, int32_t rtr) { return (rtr != 0) ? (h | (1 << 19)) : (h & ~(1U << 19)); }
inline int32_t can_header_isrtr(uint32_t h) { return ((h & (1 << 19)) != 0) ? -1 : 0; }

// from29 creates an extended header in our 32 bit representation from a 29 (id_a+id_b). RTR is clear.
inline uint32_t can_header_from29(uint32_t id29) { return ((id29 & (0x7ff << 18)) << 2)| (1 << 18) | (id29 & 0x3ffff); }

// to29 extracts the 29 bit id_a+id_b from a header.
inline uint32_t can_header_to29(uint32_t header) { return ((header >> 2) & (0x7ff << 18)) | (header & 0x3ffff); }

struct CanMsg {
    uint32_t id;
    uint32_t eid;
    uint8_t  data[8];
};

inline void mkExtCanMsg(struct CanMsg* msg, uint32_t header, size_t len, uint8_t* buf) {
    msg->id = can_header_id_a(header);
    msg->eid = (3<<28) | (can_header_id_b(header) << 10) | (len & 0xf) | (can_header_isrtr(header) ? 2 : 0);
    for (size_t i = 0; i < len; i++){
        msg->data[i] = buf[i];
    }
}

inline void mkCanMsg(struct CanMsg* msg, uint32_t header, size_t len, uint8_t* buf) {
    msg->id = can_header_id_a(header);
    msg->eid =  (len & 0xf) | (can_header_isrtr(header) ? 2 : 0);
    for (size_t i = 0; i < len; i++){
        msg->data[i] = buf[i];
    }
}

inline uint32_t canMsgHeader(struct CanMsg* msg) { 
    if (msg->eid & (1<<28)) {
        return can_header_ext(msg->id, msg->eid >> 10, msg->eid & 2);
    }
    return can_header(msg->id, msg->eid & 2);
}

inline size_t canMsgLen(struct CanMsg* msg) { return msg->eid & 0xf; }


// Sysclk is 120, 12 quanta/bit -> 10MBps/2*(1+div)
enum CANBaudRate {
    CAN_1MBd   = 4,
    CAN_500KBd = 9,
    CAN_250KBd = 19,
    CAN_125KBd = 39,
    CAN_83KBd  = 59,
};


//void c1init(enum CANBaudRate bps, struct CanMsg* fifos);
//void c2init(enum CANBaudRate bps, struct CanMsg* fifos);
void c3init(enum CANBaudRate bps, struct CanMsg* fifos);
//void c4init(enum CANBaudRate bps, struct CanMsg* fifos);

inline void * PA_TO_KVA1(uintptr_t pa) { return (void *)(pa | 0xa0000000UL); }

inline struct CanMsg *c1_tx_head() { return C1FIFOINT0bits.TXNFULLIF  ? PA_TO_KVA1(C1FIFOUA0) : NULL; }
inline struct CanMsg *c2_tx_head() { return C2FIFOINT0bits.TXNFULLIF  ? PA_TO_KVA1(C2FIFOUA0) : NULL; }
inline struct CanMsg *c3_tx_head() { return C3FIFOINT0bits.TXNFULLIF  ? PA_TO_KVA1(C3FIFOUA0) : NULL; }
inline struct CanMsg *c4_tx_head() { return C4FIFOINT0bits.TXNFULLIF  ? PA_TO_KVA1(C4FIFOUA0) : NULL; }

inline struct CanMsg *c1_rx_tail() { return C1FIFOINT1bits.RXNEMPTYIF ? PA_TO_KVA1(C1FIFOUA1) : NULL; }
inline struct CanMsg *c2_rx_tail() { return C2FIFOINT1bits.RXNEMPTYIF ? PA_TO_KVA1(C2FIFOUA1) : NULL; }
inline struct CanMsg *c3_rx_tail() { return C3FIFOINT1bits.RXNEMPTYIF ? PA_TO_KVA1(C3FIFOUA1) : NULL; }
inline struct CanMsg *c4_rx_tail() { return C4FIFOINT1bits.RXNEMPTYIF ? PA_TO_KVA1(C4FIFOUA1) : NULL; }

inline void c1_tx_push(void) { C1FIFOCON0SET = _C1FIFOCON0_UINC_MASK | _C1FIFOCON0_TXREQ_MASK; }
inline void c2_tx_push(void) { C2FIFOCON0SET = _C2FIFOCON0_UINC_MASK | _C2FIFOCON0_TXREQ_MASK; }
inline void c3_tx_push(void) { C3FIFOCON0SET = _C3FIFOCON0_UINC_MASK | _C3FIFOCON0_TXREQ_MASK; }
inline void c4_tx_push(void) { C4FIFOCON0SET = _C4FIFOCON0_UINC_MASK | _C4FIFOCON0_TXREQ_MASK; }

inline void c1_rx_pull(void) { C1FIFOCON1SET = _C1FIFOCON1_UINC_MASK; }
inline void c2_rx_pull(void) { C2FIFOCON1SET = _C2FIFOCON1_UINC_MASK; }
inline void c3_rx_pull(void) { C3FIFOCON1SET = _C3FIFOCON1_UINC_MASK; }
inline void c4_rx_pull(void) { C4FIFOCON1SET = _C4FIFOCON1_UINC_MASK; }

