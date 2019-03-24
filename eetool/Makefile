CC ?= gcc
RM = rm -f

CCFLAGS = -Wall -fpack-struct

OBJS = eetool.o ../lipoCcounter/crc.o

all: eetool

%.o : %.c
	$(CC) -c $(CCFLAGS) $(DEPFLAGS_CC) $< -o $@

eetool: $(OBJS)
	$(CC) $(CCFLAGS) $^ -o eetool

clean:
	$(RM) $(OBJS)
	$(RM) eetool

.PHONY: all clean
