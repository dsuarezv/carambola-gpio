
include Makefile.dave

CFLAGS	+= -Wall -Os -g -Wno-unused-function
OBJS     = libgpio.o test.o
OUTPUT   = testgpio

STAGING_DIR := $(STAGING_DIR)
export STAGING_DIR

all: test

test: $(OBJS)
	@$(CC) -o $(OUTPUT) $(OBJS) $(LDFLAGS)

%.o : %.c
	@$(CC) -c $(CFLAGS) $<

clean:
	@rm -f $(OBJS) $(OUTPUT)