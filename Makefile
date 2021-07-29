# Tool names
PREFIX=xc32-
CC          := $(PREFIX)gcc
BIN2HEX     := $(PREFIX)bin2hex

APPLICATION := $(shell basename $(CURDIR))

ARCH_FLAGS      = -mprocessor=32MK1024GPE100
WARN_FLAGS      = -Werror -Wfatal-errors -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion -Wundef
DEBUG_FLAGS     = -DNDEBUG
CFLAGS          = -O2 $(ARCH_FLAGS) $(WARN_FLAGS) $(DEBUG_FLAGS)
LDFLAGS         = $(ARCH_FLAGS) $(WARN_FLAGS) $(DEBUG_FLAGS)

default:$(APPLICATION).hex

SRCS := $(wildcard *.c)
OBJS := $(SRCS:.c=.o)

$(OBJS): Makefile

# Compile
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(APPLICATION).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

%.hex: %.elf
	$(BIN2HEX) $<

clean:
	rm -f *~ *.o *.hex *.elf

depend:
	makedepend -Y. -w150 *.c
# DO NOT DELETE

main.o: printf.h
printf.o: printf.h stb_sprintf.h
