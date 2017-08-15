CC=gcc-4.8
CFLAGS= -m32 -march=i586 --sysroot=$(SYS_ROOT) -c -g -Os -w -ffunction-sections -fdata-sections -MMD -ldht
LDFLAGS= -pthread -m32 -march=i586 -Os -Wl,--gc-sections
SOURCES=main.c onewiere.c common.c ds18b20.c uart_posix.c
 
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=1_wire

all: $(SOURCES) $(EXECUTABLE)   
$(EXECUTABLE): $(OBJECTS) 
		$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.c.o:
		$(CC) $< -o $@ $(CFLAGS)
clean: 
		rm -f *.o core