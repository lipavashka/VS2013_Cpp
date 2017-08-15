//
//  OWUart.c
//  OWUart
//
//  Created by Kenny Milar on 1/June/2014.
//  Copyright (c) 2014 SpiderElectron. All rights reserved.
//

#include <stdio.h> 
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "onewire.h"
#include "ds18b20.h"
#include "common.h"

#define DEVICE_FAMILY_IBUTTON 0x01
#define const float pii = 3.14;

uint8_t id[OW_ROMCODE_SIZE];

char *get_type_by_id(uint8_t id) {
	switch (id) {
	case DS18S20_FAMILY_CODE: return "DS18S20";
	case DS18B20_FAMILY_CODE: return "DS18B20";
	case DS1822_FAMILY_CODE: return "DS1822";
	default: return "WTF?!";
	}
}

// new function
uint8_t id[OW_ROMCODE_SIZE];

/* errExit - helper function, log message and quit. */
void errExit(char *p)
{
	if (p)
	{
		printf("Exiting due to: %s\n", p);
	}
	else
	{
		printf("Error. Exit. Sorry.\n");
	}
	exit(-1);
}

/* Set specified GPIO pin for use. */
void setGPIOPin(char* pin, char* dir, char* drive, char* val)
{
	char buf[256];
	int fd;

	// Open the GPIO Export file
	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1)
		errExit("GPIO Export");

	// Export the required pin.
	write(fd, pin, strlen(pin)); // Export GPIO pin
	close(fd);

	// Open exported pin's DIRECTION file
	sprintf(buf, "/sys/class/gpio/gpio%s/direction", pin);
	fd = open(buf, O_WRONLY); // open GPIOxx direction file
	if (fd == -1)
		errExit("Gpio Direction");

	// write out the direction
	write(fd, dir, strlen(dir)); // set GPIOxx direction to out
	close(fd);

	// open the drive file
	sprintf(buf, "/sys/class/gpio/gpio%s/drive", pin);
	fd = open(buf, O_WRONLY);
	if (fd == -1)
		errExit("Gpio Drive");

	// Write the drive type.
	write(fd, drive, strlen(drive)); // set GPIO drive
	close(fd);

	// Open the initial value file.
	sprintf(buf, "/sys/class/gpio/gpio%s/value", pin);
	fd = open(buf, O_WRONLY);
	if (fd == -1)
		errExit("Gpio Value");

	write(fd, val, strlen(val)); // set GPIO initial value
	close(fd);

}

/*
setMux - set up the multiplexor on A0 to connect it to ADC VIN0
*/
void setMux(void)
{
	setGPIOPin("40", "out", "strong", "0"); // ttyS0 connects to RX (Arduino 0)
	setGPIOPin("41", "out", "strong", "0"); // ttyS0 to TX	(Arduino 1)
	setGPIOPin("4", "out", "strong", "1"); // Level shifter enabled (enable the final driver).
}

/* Open a file descripto to the specified path */
int open_port(char *path)
{
	int fd; /* File descriptor for the port */


	fd = open(path, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		perror("open_port: Unable to open specified port. ");
	}
	else
		fcntl(fd, F_SETFL, 0); // ASYNC access.

	return (fd);
}

int setBaud(int fd, unsigned int baudrate)
{
	struct termios options;

	// Get the current port options.
	tcgetattr(fd, &options);

	// Set the baud rates to one of our favourite bauds
	if (baudrate == 300)
	{
		cfsetispeed(&options, B300);
		cfsetospeed(&options, B300);

	}
	else if (baudrate == 600)
	{
		cfsetispeed(&options, B600);
		cfsetospeed(&options, B600);

	}
	else if (baudrate == 1200)
	{
		cfsetispeed(&options, B1200);
		cfsetospeed(&options, B1200);

	}
	else if (baudrate == 2400)
	{
		cfsetispeed(&options, B2400);
		cfsetospeed(&options, B2400);

	}
	else if (baudrate == 4800)
	{
		cfsetispeed(&options, B4800);
		cfsetospeed(&options, B4800);

	}

	else if (baudrate == 9600)
	{
		cfsetispeed(&options, B9600);
		cfsetospeed(&options, B9600);

	}
	else if (baudrate == 57600)
	{
		cfsetispeed(&options, B57600);
		cfsetospeed(&options, B57600);
	}
	else if (baudrate == 115200)
	{
		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);
	}
	else
	{
		printf("I didn't bother setting the port speed.\n");
	}

	// Enable the receiver and set local mode...
	options.c_cflag |= (CLOCAL | CREAD);

	// 8N1
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	// No flow control
	options.c_cflag &= ~CRTSCTS;
	options.c_iflag &= ~(IXON | IXOFF | IXANY);

	// raw input
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// raw output
	options.c_oflag &= ~OPOST;

	// Set the new options for the port
	tcsetattr(fd, TCSADRAIN, &options);

	// No waiting for characters
	fcntl(fd, F_SETFL, FNDELAY);

	return 0;
}

// Write a buffer of data to the port.
int write_port(int fd, unsigned char *pData, int len)
{
	int bytesWritten;

	bytesWritten = write(fd, pData, len);
	printf("Write of %d bytes returned %d\n",len,bytesWritten);

	return bytesWritten;

}

// Read from the port into a buffer of data.
int read_port(int fd, unsigned char *buf, int len)
{
	// int i, bytesRead;
	int bytesRead;
	int totalBytes;

	totalBytes = 0;

	while (1) {
		bytesRead = read(fd, buf, len);

		if (bytesRead <= 0)
			break;

		totalBytes += bytesRead;

	}

	return totalBytes;
}


// 1-Wire protocol byte write.
// You must understand the 1-Wire protocol to see whats happening here.
// We are using the UART to create pulses NOT to send actual bytes.
// A short pulse is a '1' bit and a longer pulse is a '0' bit.
// so we use the start bit, followed by 0 or more '0' bits to
// send a pulse of the requried length.
//
// Effectively we are runing a software UART over a faster hardware UART.
//
void OW_WriteByte(int fd, unsigned char dataByte)
{
	int i, res;
	unsigned char out;
	unsigned char inBuf[64]; // for flushin our echos

	setBaud(fd, 115200);

	for (i = 0; i<8; i++)
	{
		// Are we sending a '1' or a '0' for this bit?
		out = (dataByte & 0x01) ? 0xff : 0xc0;
		res = write(fd, &out, 1);

		if (res == -1) // check for failure.
		{
			printf("Failed to write (%d)\n", i);
			exit(-1);
		}

		dataByte >>= 1; // shift next bit into position.
	}

	// Since TX and RX are shorted, we must flush our output from the inbuffer
	// where it will have been received. We read our own writes.
	while (1)
	{
		i = read(fd, inBuf, 64);
		if (i <= 0)
			break;

		//		printf("Flushed %d bytes\n",i);
	}

}

// Using the UART to create a read time slot, then
// detecting if the iButton created a 0-pulse or not.
// Effectively creating a software uart over the hardware uart.
int OW_ReadBytes(int fd, unsigned char* pBuf, int len)
{
	int i;
	int bits, bytes;

	unsigned char startSlot = 0xff;
	unsigned char inBuf[64];
	unsigned char newByte;

	// Read len bytes from the bus, 1-Bit at a time.
	for (bytes = 0; bytes < len; bytes++)
	{
		newByte = 0;
		for (bits = 0; bits<8; bits++)
		{
			newByte >>= 1;

			write(fd, &startSlot, 1);
			usleep(500); // time for byte to exit

			i = read(fd, inBuf, 64);
			if (i <= 0)
			{
				//				printf("End read\n");
				return bytes;
			}


			if (inBuf[0] == 0xff) // iButton did not bring line down at all.
			{
				newByte |= 0x80;
			}

		}
		pBuf[bytes] = newByte;
	}

	return bytes;
}

unsigned char OW_CRC(unsigned char *pBuf, int len)
{
	unsigned char loop, i, bit;
	unsigned char crc = 0x00;

	for (loop = 0; loop<len; loop++)
	{
		crc = (crc ^ pBuf[loop]);

		for (i = 8; i>0; i--)
		{
			bit = (crc & 0x01);
			crc >>= 1;

			if (bit)
			{
				crc = (crc ^ 0x8c);
			}
		}
	}

	return crc;
}


int OW_detectKey(int fd)
{
	int bytesRead;
	unsigned char out;
	unsigned char inBuf[64];

	setBaud(fd, 9600);
	out = 0xF0;
	write_port(fd, &out, 1);
	//usleep(1000 * 10);
	sleep(1);
	bytesRead = read_port(fd, inBuf, 64);

	if (bytesRead == 1) {
		if (inBuf[0] == 0xe0)
		{
			printf("Key detected\n");
			return 1;
		}
		else
		{
			printf("Error: Key not detected. Read: 0x%X\n", inBuf[0]);
		}
	}
	else
	{
		printf("Error: bytesRead != 1; %X\n", bytesRead);
	}
	return 0;
}


// from AVR

/*
* ds18b20 init
*/
uint8_t ds18b20_reset(int _fd) {
	uint8_t i = 1;

	if (OW_detectKey(_fd)) // is there a key on the probe?
	{
		return 0;
	}
	else
	{
		return i;
	}

	//return the read value, 0=ok, 1=error
	// return i;
}

// write one byte
void ds18b20_writebyte(int _fd, uint8_t byte){
	OW_WriteByte(_fd, byte);
}

// read one byte
uint8_t ds18b20_readbyte(int _fd){
	uint8_t i = 8;
	int n = 0;
	unsigned char pBuf[2] = { 0, 0 };

	n = OW_ReadBytes(_fd, pBuf, 1);

	return (unsigned char)n;
}

void UART_putc(int _fd, unsigned char c) {
	if (_fd < 0) return;
	write(_fd, &c, 1);
}

unsigned char UART_getc(int _fd) {
	if (_fd < 0) return 0x00;
	static unsigned char c = 0x00;
	while (!read(_fd, &c, 1));
	return c;
}

uint8_t OW_bit(int _fd, uint8_t b) {
	uint8_t c;

	if (b) UART_putc(_fd, 0xff); /* Write 1 */
	else  UART_putc(_fd, 0x00);  /* Write 0 */

	/* Read */
	c = UART_getc(_fd);

#ifdef DEBUG
	printf("DEBUG: ATOMIC OP: %d -> %d\n", b, c == 0xff);
#endif

	return (c == 0xff);
}


uint8_t DS18X20_conversion_in_Progress(int _fd) {
	return OW_bit(_fd, 1) ? DS18X20_CONVERSION_DONE : DS18X20_CONVERTING;
}

// get temperature

double ds18b20_gettemp(int _fd) {
	uint8_t temperature[2];
	int8_t digit;
	uint16_t decimal;
	double retd = 0;

	printf("ds18b20_reset \n");
	ds18b20_reset(_fd); //reset
	ds18b20_writebyte(_fd, DS18B20_CMD_SKIPROM); //skip ROM
	ds18b20_writebyte(_fd, DS18B20_CMD_CONVERTTEMP); //start temperature conversion

	DS18X20_conversion_in_Progress(_fd); //wait until conversion is complete

	ds18b20_reset(_fd); //reset
	ds18b20_writebyte(_fd, DS18B20_CMD_SKIPROM); //skip ROM
	ds18b20_writebyte(_fd, DS18B20_CMD_RSCRATCHPAD); //read scratchpad

	//read 2 byte from scratchpad
	temperature[0] = ds18b20_readbyte(_fd);
	temperature[1] = ds18b20_readbyte(_fd);

	ds18b20_reset(_fd); //reset

	//store temperature integer digits
	digit = temperature[0] >> 4;
	digit |= (temperature[1] & 0x7) << 4;

	//store temperature decimal digits
	decimal = temperature[0] & 0xf;
	decimal *= DS18B20_DECIMALSTEPS;

	//compose the double temperature value and return it
	retd = digit + decimal * 0.0001;

	return retd;
}


int main(int argc, char *argv[])
{
	int fd;
	int i;
	const int max_read_data = 8;
	// int i, bytesRead;

	// unsigned char buf[64] = { 0, };
	unsigned char inBuf[512] = { 0, };
	unsigned char crc;

	// new function
	uint8_t c = 0, diff = OW_SEARCH_FIRST;
	int16_t temp_dc;

	setMux();

	if (argc != 2)
	{
		// printf("Usage: %s <path to serialPort>\n", argv[0]);
		// exit(-1);
		argv[1] = "/dev/ttyQRK0"; // ttyQRK0 ttyS0
	}

	fd = open_port(argv[1]);
	// fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
	if (fd == -1)
	{
		printf("Failed to open serial port at %s\n", argv[1]);
		exit(-1);
	}

	while (1) //one infinte loop
	{
		if (OW_detectKey(fd)) // is there a key on the probe?
		{
			OW_WriteByte(fd, 0x33); // 0x33 // if so, write the 'read rom' command to it.
			printf("OW_WriteByte(fd, 0x33)\n");
			if (OW_ReadBytes(fd, inBuf, max_read_data) == max_read_data) // then try to read back 8 bytes.
			{
				crc = OW_CRC(inBuf, max_read_data);
				if (inBuf[0] == DEVICE_FAMILY_IBUTTON)	// 1st byte is device family. We want device family 1.
				{
					if (crc == inBuf[max_read_data-1])		// If CRC matches the last byte in the buffer
					{
						printf("Valid key detected with ID:");
						printf("%02x%02x%02x%02x%02x%02x\n", inBuf[6], inBuf[5], inBuf[4], inBuf[3], inBuf[2], inBuf[1]);
						sleep(1); // Wait 1 second before trying again.
					}
				}
				else
				{
					if (crc == inBuf[max_read_data - 1])		// If CRC matches the last byte in the buffer
					{
						printf("CRC Coorect: 0x%X==0x%X\n", inBuf[max_read_data - 1], crc);
					}
					else
					{
						printf("Error CRC: 0x%X!=0x%X\n", inBuf[max_read_data - 1], crc);
						crc = crc8(inBuf, max_read_data);
						printf("New CRC: 0x%X\n", crc);
					}
					printf("Not Valid key detected with ID: ");
					for (i = 0; i < max_read_data; i++)
					{
						printf("%02x", inBuf[i]);
					}
					printf("\n");
					// printf("%02x%02x%02x%02x%02x%02x\n", inBuf[6], inBuf[5], inBuf[4], inBuf[3], inBuf[2], inBuf[1]);
					sleep(1); // Wait 1 second before trying again.
				}
			}
			else
			{
				printf("Data < 8 bytes, Data:");
				printf("%02x%02x%02x%02x%02x%02x\n", inBuf[6], inBuf[5], inBuf[4], inBuf[3], inBuf[2], inBuf[1]);
				sleep(1); // Wait 1 second before trying again.
			}

			//// try to read temperature
			//sleep(1);
			//printf("ds18b20: \n");
			//printf("Temp: %d\n", ds18b20_gettemp(fd));
		}

		//// usleep(1000 * 100);

		//close(fd);

		sleep(1);

		// new function 
		argv[1] = "/dev/ttyQRK0";
		if (ow_init(argv[1])) {
			puts("Bus INIT failed. Check COM port.\n");
			return 1;
		}
		for (i = 0; i < sizeof(id); i++)
		{
			id[i] = 0;
		}
		printf("--------------------------------------------------------------\n");
		printf("diff: %d\n", diff);
		
		c = 0;
		while (diff != OW_LAST_DEVICE) {
			diff = ow_rom_search(diff, id);
			if (diff == OW_ERR_PRESENCE) 
			{
				puts("All devices are offline now.\n");
				ow_finit();
				return 1;		 
			}
			if (diff == OW_ERR_DATA) {
				puts("Bus error.\n");
				ow_finit();
				return 1;
			}
			printf("RAW Data: ");
			for (i = 0; i < sizeof(id); i++)
			{
				printf("%02x", id[i]);
			}
			printf("\n");
			printf("Bus %s Device %03u Type 0x%02hx ID %02hx%02hx%02hx%02hx%02hx%02hx CRC 0x%02hx\n", \
				argv[1], c, id[0], id[6], id[5], id[4], id[3], id[2], id[1], id[7]);
			c++;
		}
		puts("Bus listed.\n");		 
		// ow_finit();

		c = 0;
		diff = OW_SEARCH_FIRST;
		while (diff != OW_LAST_DEVICE) {
			DS18X20_find_sensor(&diff, id);
			if (diff == OW_ERR_PRESENCE) {
				puts("All sensors are offline now.\n");
				ow_finit();
				return 1;
			}
			if (diff == OW_ERR_DATA) {
				puts("Bus error.\n");
				ow_finit();
				return 1;
			}
			printf("Bus %s Device %03u Type 0x%02hx (%s) ID %02hx%02hx%02hx%02hx%02hx%02hx CRC 0x%02hx ", \
				argv[1], c, id[0], get_type_by_id(id[0]), id[6], id[5], id[4], id[3], id[2], id[1], id[7]);
			c++;

			if (DS18X20_start_meas(DS18X20_POWER_EXTERN, NULL) == DS18X20_OK) {
				if (DS18X20_read_decicelsius(id, &temp_dc) == DS18X20_OK) {
					printf("TEMP %3d.%01d C\n", temp_dc / 10, temp_dc > 0 ? temp_dc % 10 : -temp_dc % 10);
					continue;
				}
			} 

			puts("MEASURE FAILED!\n");

		}
		puts("Sensors listed.\n");
		diff = OW_SEARCH_FIRST;

	}

	// close(fd);
}


/* end of file */
