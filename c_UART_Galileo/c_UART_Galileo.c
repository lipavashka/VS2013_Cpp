#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
#include <stdlib.h>
#include <string.h>  /* String function definitions */


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

void setMux(void)
{
	setGPIOPin("40", "out", "strong", "0"); // ttyS0 connects to RX (Arduino 0)
	setGPIOPin("41", "out", "strong", "0"); // ttyS0 to TX	(Arduino 1)
	setGPIOPin("4", "out", "strong", "1"); // Level shifter enabled (enable the final driver).
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
int write_port(int fd, char *pData, int len)
{
	int bytesWritten;

	bytesWritten = write(fd, pData, len);
	printf("Write of %d bytes returned %d\n", len, bytesWritten);

	return bytesWritten;

}

// Read from the port into a buffer of data.
int read_port(int fd, char *buf, int len)
{
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

int main()
{
	int fd;
	char * const port = "/dev/ttyQRK0";
	char write_buffer[] = "sdjfnjsdnfjksdfnsjkdfnsjdfnkfnd"; // test data
	char read_buffer[32];
	int bytesRead;

	setMux();

	fd = open(port, O_RDWR | O_NOCTTY);
	if (fd == 1)
	{
		printf("\n  Error! in Opening %s\n", port);
	}
	else
	{
		printf("\n  %s Opened Successfully\n", port);
	}

	setBaud(fd, 115200);

	// Write data
	write_port(fd, write_buffer, sizeof(write_buffer));

	usleep(10000);

	// Read Data
	bytesRead = read_port(fd, read_buffer, sizeof(read_buffer));

	printf("bytesRead: %X, Data = %s\n", bytesRead, read_buffer);

	close(fd);
	return 0;
}
