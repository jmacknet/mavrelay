SOURCES=mav-relay.c
LIBS=
CFLAGS=-O3
LDFLAGS=

# determine object filenames
OBJS := $(SOURCES:.c=.o)

# link
mav-relay: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o mav-relay

# pull in dependency info for *existing* .o files
-include $(OBJS:.o=.d)

# compile and generate dependency info
%.o: %.c
	$(CC) -c $(CFLAGS) $*.c -o $*.o
	$(CC) -MM $(CFLAGS) $*.c > $*.d

# clean build files
clean:
	rm -f *.o *.d core mav-relay
