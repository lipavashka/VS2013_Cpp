CC=gcc-4.8
CFLAGS= -m32 -march=i586 --sysroot=$(SYS_ROOT) -c -g -Os -w -ffunction-sections -fdata-sections -MMD -ldht
LDFLAGS= -pthread -m32 -march=i586 -Os -Wl,--gc-sections
SOURCES=c_AM2301
 
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=AM2301

all: $(SOURCES) $(EXECUTABLE)   
$(EXECUTABLE): $(OBJECTS) 
		$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.c.o:
		$(CC) $< -o $@ $(CFLAGS)
clean: 
		rm -f *.o core