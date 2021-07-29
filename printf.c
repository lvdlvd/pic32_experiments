#include "printf.h"

// definitions are required one compilation unit
// _full tests for free < 2 instead of == 0 so that there's room for the final '\n' or 0x7e
inline uint16_t ringbuffer_avail(struct Ringbuffer *rb);
inline uint16_t ringbuffer_free(struct Ringbuffer *rb);
inline int      ringbuffer_empty(struct Ringbuffer *rb);
inline void     ringbuffer_clear(struct Ringbuffer *rb);
inline int      ringbuffer_full(struct Ringbuffer *rb);
inline void     put_head(struct Ringbuffer *rb, uint8_t c);
inline uint8_t  get_tail(struct Ringbuffer *rb);

size_t ringbuffer_puts(struct Ringbuffer *rb, const char *buf, size_t len) {
    if (ringbuffer_free(rb) < len) {
        len = ringbuffer_free(rb);
    }
    for (size_t i = 0; i < len; ++i) {
        put_head(rb, buf[i]);
    }
    return len;
}

#define STB_SPRINTF_STATIC
#define STB_SPRINTF_MIN 32
#define STB_SPRINTF_NOFLOAT
#define STB_SPRINTF_IMPLEMENTATION

#include "stb_sprintf.h"

// a little signature adapter
static char *rb_putcb(char *buf, void *user, int len) {
    puts_t *callback = (puts_t *)user;
    size_t  ln       = len;  // explicit cast
    if (callback(buf, len) < ln) {
        return NULL;
    }
    return buf;
}

int cbprintf(puts_t *callback, const char *fmt, ...) {
    stbsp_set_separators('\'', '.');
    va_list ap;
    va_start(ap, fmt);
    char b[STB_SPRINTF_MIN];
    int  rv = stbsp_vsprintfcb(rb_putcb, callback, b, fmt, ap);
    va_end(ap);
    return rv;
}
