CC ?= cc
CPPFLAGS += $(shell pkg-config --cflags portaudio-2.0 zlib openssl)
CFLAGS ?= -O2
CFLAGS += -std=c11 -Wall -Wextra -Wpedantic
LDLIBS += $(shell pkg-config --libs portaudio-2.0 zlib openssl)

.PHONY: all clean

all: tx rx

tx: tx.c smod.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tx.c $(LDLIBS)

rx: rx.c smod.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ rx.c $(LDLIBS)

clean:
	$(RM) tx rx
