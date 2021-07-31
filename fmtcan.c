
#include "can.h"
#include "fmtcan.h"

// hex returns the character value of the lower nibble of v.
static char hex(uint32_t v) { return "0123456789abcdef"[v & 0xf]; }

// put_char puts the lowest 8 bits of c at *buf if there is place, decrements
// *left, returning a non-zero value if there was no place, zero otherwise.
static int put_char(char **buf, size_t *left, char c) {
	if (*left == 0) {
		return -1;
	}
	**buf = c;
	++(*buf);
	--(*left);
	return 0;
}

size_t can_fmt(char *buf, size_t sz, uint32_t header, size_t len, const uint8_t *payload) {
	if (len > 8) {
		return 0;
	}

	char * b    = buf;
	size_t left = sz;

	const uint32_t ida = can_header_id_a(header);
	if (put_char(&b, &left, hex(ida >> 8))) {
		return 0;
	}
	if (put_char(&b, &left, hex(ida >> 4))) {
		return 0;
	}
	if (put_char(&b, &left, hex(ida))) {
		return 0;
	}

	if (can_header_isext(header)) {
		if (put_char(&b, &left, '.')) {
			return 0;
		}
		const uint32_t idb = can_header_id_b(header);
		if (put_char(&b, &left, hex(idb >> 16))) {
			return 0;
		}
		if (put_char(&b, &left, hex(idb >> 12))) {
			return 0;
		}
		if (put_char(&b, &left, hex(idb >> 8))) {
			return 0;
		}
		if (put_char(&b, &left, hex(idb >> 4))) {
			return 0;
		}
		if (put_char(&b, &left, hex(idb))) {
			return 0;
		}
	}

	if (can_header_isrtr(header)) {
		if (put_char(&b, &left, 'R')) {
			return 0;
		}
	}

	if ((len > 0) && (payload != NULL)) {
		if (put_char(&b, &left, ':')) {
			return 0;
		}

		for (size_t i = 0; i < len; i++) {
			if (put_char(&b, &left, hex(payload[i] >> 4))) {
				return 0;
			}
			if (put_char(&b, &left, hex(payload[i]))) {
				return 0;
			}
		}
	}

	if (put_char(&b, &left, 0)) {
		return 0;
	}

	return sz - left;
}

// verified in https://play.golang.org/p/UWR_B6opEP
// used here and in hdlcan.c
uint16_t can_crc16_next(const uint16_t crc_in, uint8_t data) {
	uint16_t crc = crc_in;
	crc ^= (uint16_t)data << 8;
	for (int i = 0; i < 8; i++) {
		if ((crc & 0x8000) != 0) {
			crc ^= 0xc599;
		}
		crc <<= 1;
	}
	return crc;
}

size_t can_fmt_crc(char *buf, size_t sz, uint32_t header, size_t len, const uint8_t *payload) {
	size_t n = can_fmt(buf, sz, header, len, payload);
	if ((n == 0) || (n + 5 > sz)) {
		return 0;
	}
	char * b    = buf + n - 1; // backup over the NUL
	size_t left = sz - n + 1;

	uint16_t crc = 0;
	crc          = can_crc16_next(crc, (uint8_t)(header >> 24));
	crc          = can_crc16_next(crc, (uint8_t)(header >> 16));
	crc          = can_crc16_next(crc, (uint8_t)(header >> 8));
	crc          = can_crc16_next(crc, (uint8_t)(header));
	for (size_t i = 0; i < len; ++i) {
		crc = can_crc16_next(crc, payload[i]);
	}

	// for empty payload add the first colon now.
	if ((len == 0) || (payload == NULL)) {
		if (put_char(&b, &left, ':')) {
			return 0;
		}
	}

	if (put_char(&b, &left, ':')) {
		return 0;
	}
	if (put_char(&b, &left, hex(crc >> 12))) {
		return 0;
	}
	if (put_char(&b, &left, hex(crc >> 8))) {
		return 0;
	}
	if (put_char(&b, &left, hex(crc >> 4))) {
		return 0;
	}
	if (put_char(&b, &left, hex(crc))) {
		return 0;
	}
	if (put_char(&b, &left, 0)) {
		return 0;
	}

	return sz - left;
}

// get_char returns the next character from buf or -1 if *left is zero,
// updating buf and left.
static int32_t get_char(const char **buf, size_t *left) {
	if (*left == 0) {
		return -1;
	}
	int32_t c = (unsigned char)(**buf);
	++(*buf);
	--(*left);
	return c;
}

// unhex takes the ascii value of c (0..0x7f) and returns the corresponding hex value
// or 0xffff if c is not a hex character.
static uint32_t unhex(int32_t c) {
	switch (c) {
	case '0':
		return 0x0;
	case '1':
		return 0x1;
	case '2':
		return 0x2;
	case '3':
		return 0x3;
	case '4':
		return 0x4;
	case '5':
		return 0x5;
	case '6':
		return 0x6;
	case '7':
		return 0x7;
	case '8':
		return 0x8;
	case '9':
		return 0x9;
	case 'A':
	case 'a':
		return 0xA;
	case 'B':
	case 'b':
		return 0xB;
	case 'C':
	case 'c':
		return 0xC;
	case 'D':
	case 'd':
		return 0xD;
	case 'E':
	case 'e':
		return 0xE;
	case 'F':
	case 'f':
		return 0xF;
	}
	return 0xffff;
}

// scan_hex reads characters from (*buf)
// stopping when *left is zero, or 8 characters have been read or
// **buf is not a hexadecimal character.
static uint32_t scan_hex(const char **buf, size_t *left) {
	uint32_t v = 0;

	for (size_t i = 0; i < 8; ++i) {
		if (*left == 0) {
			break;
		}

		const int32_t  c = (unsigned char)(**buf);
		const uint32_t d = unhex(c);
		if (d > 0xf) {
			break;
		}
		++(*buf);
		--(*left);

		v <<= 4;
		v |= d;
	}

	return v;
}

static int32_t is_alnum(int32_t c) {
	if (('0' <= c) && (c <= '9')) {
		return 1;
	}
	if (('A' <= c) && (c <= 'Z')) {
		return 1;
	}
	if (('a' <= c) && (c <= 'z')) {
		return 1;
	}
	return 0;
}

// all pointer arguments must be non-null
size_t can_scan(const char *buf, size_t sz, uint32_t *header, size_t *len, uint8_t *payload) {
	const char *b    = buf;
	size_t      left = sz;

	*header = 0;

	// ida may not be zero, so the empty hex string is also not allowed.
	const uint32_t ida = scan_hex(&b, &left);
	if ((ida == 0) || ida > 0x7ff) {
		return 0;
	}

	*header = can_header(ida, 0);

	if ((left > 1) && (*b == '.')) {
		++b;
		--left;
		// TODO this also accepts the empty hex string, indistinguishable from 0000...
		const uint32_t idb = scan_hex(&b, &left);
		if (idb > 0x3ffff) {
			return 0;
		}
		*header = can_header_ext(ida, idb, 0);
	}

	if ((left > 0) && (*b == 'R')) {
		++b;
		--left;
		*header = can_header_setrtr(*header, 1);
	}

	if ((left > 1) && (*b == ':')) {
		++b;
		--left;

		for (*len = 0; *len < 8; ++(*len)) {
			{
				const int32_t c = get_char(&b, &left);
				if (c < 0) {
					break;
				}
				// any non-alphanumeric terminates the payload.
				if (!is_alnum(c)) {
					++left;  // unconsume it.
					break;
				}
				// but if it is an alphanumeric, it better be a valid hex digit.
				const uint32_t d = unhex(c);
				if (d > 0xf) {
					return 0;
				}
				payload[*len] = (uint8_t)(d << 4);
			}

			// we're in the middle of a hex byte, so the 2nd nibble is mandatory
			{
				const int32_t c = get_char(&b, &left);
				if (c < 0) {
					return 0;
				}
				const uint32_t d = unhex(c);
				if (d > 0xf) {
					return 0;
				}
				payload[*len] |= (uint8_t)(d);
			}
		}
	}

	return sz - left;
}

size_t can_scan_crc(const char *buf, size_t sz, uint32_t *header, size_t *len, uint8_t *payload) {
	const size_t n = can_scan(buf, sz, header, len, payload);
	if (n == 0) {
		return 0;
	}
	const char *b    = buf + n ;
	size_t      left = sz - n;

	if (left < 5) {
		return 0;
	}

	// consume the ':'
	if (*b != ':') {
		return 0;
	}
	++b;
	--left;

	// parse 6 hex digits
	uint32_t crc_in = 0;
	for (int i = 0; i < 4; ++i) {
		const int32_t c = get_char(&b, &left);
		if (c < 0) {
			return 0;
		}
		const uint32_t d = unhex(c);
		if (d > 0xf) {
			return 0;
		}
		crc_in <<= 4;
		crc_in |= d;
	}

	// check the crc by adding in the check, should come out to zero
	uint16_t crc = 0;
	crc          = can_crc16_next(crc, (uint8_t)(*header >> 24));
	crc          = can_crc16_next(crc, (uint8_t)(*header >> 16));
	crc          = can_crc16_next(crc, (uint8_t)(*header >> 8));
	crc          = can_crc16_next(crc, (uint8_t)(*header));
	for (size_t i = 0; i < *len; ++i) {
		crc = can_crc16_next(crc, payload[i]);
	}
	crc = can_crc16_next(crc, (uint8_t)(crc_in >> 8));
	crc = can_crc16_next(crc, (uint8_t)(crc_in));

	if (crc != 0) {
		return 0;
	}

	return sz - left;
}
