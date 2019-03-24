CC ?= gcc
RM = rm -f

CCFLAGS = -Wall

OBJS = eetool.c ../lipoCcounter/crc.c

all: eetool

%.o : %.c
	$(CC) -c $(CCFLAGS) $(DEPFLAGS_CC) $< -o $@

eetool: $(OBJS)
	$(CC) $(CCFLAGS) $^ -o eetool

clean:
	$(RM) $(OBJS)
	$(RM) eetool

.PHONY: all clean
