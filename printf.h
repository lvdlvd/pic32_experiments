#pragma once
/*
    Everything you need to implement a printf to serial or similar devices.
    The implemenation is done by stb_printf.h

    Typically you make a puts_t callback to copy to ringbuffer and enable the irq or dma 
    that empties it, and in the irq/dma handler you pull from teh ringbuffer and switch it off when empty.
*/

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

struct Ringbuffer {
    uint8_t  buf[1<<9]; // size must be power of two or '% sizeof(rb->buf)' will not be efficient
    uint16_t head;      // writes happen here
    uint16_t tail;      // reads happen here
};

inline uint16_t ringbuffer_avail(struct Ringbuffer *rb) 	{ return rb->head - rb->tail; }                         // 0..size -1
inline uint16_t ringbuffer_free(struct Ringbuffer *rb) 		{ return sizeof(rb->buf) - (rb->head - rb->tail) - 1; } // size-1 .. 0
inline int      ringbuffer_empty(struct Ringbuffer *rb) 	{ return rb->head == rb->tail; }
inline void     ringbuffer_clear(struct Ringbuffer *rb) 	{ rb->head = rb->tail; }
inline int      ringbuffer_full(struct Ringbuffer *rb) 		{ return ringbuffer_free(rb) < 2; } 					// see note in .c
inline void     put_head(struct Ringbuffer *rb, uint8_t c)	{ rb->buf[rb->head++ & (sizeof(rb->buf)-1)] = c; }
inline uint8_t  get_tail(struct Ringbuffer *rb) 			{ return rb->buf[rb->tail++ & (sizeof(rb->buf)-1)]; }

// copies as much of buf[:len] to rb as will fit, returns the number of bytes copied.
size_t ringbuffer_puts(struct Ringbuffer *rb, const char *buf, size_t len);

// type of callback called by cbprintf() repeatedly.
typedef size_t puts_t(const char *buf, size_t len);

// cbprintf() interprets fmt as a format string for the variable parameters and calls the callback to
// copy the characters out, up to 32 at a time.  When the callback returns less than len, printing
// is aborted.
int cbprintf(puts_t *callback, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
