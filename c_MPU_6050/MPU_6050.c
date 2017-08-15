
// MPU6050 code.

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <i2c/smbus.h> 
#include <unistd.h>
#include <math.h>

#define BMP085_I2C_ADDRESS 0x77

const unsigned char BMP085_OVERSAMPLING_SETTING = 3;

// Calibration values - These are stored in the BMP085
short int ac1;
short int ac2;
short int ac3;
unsigned short int ac4;
unsigned short int ac5;
unsigned short int ac6;
short int b1;
short int b2;
short int mb;
short int mc;
short int md;

int b5;

unsigned int temperature, pressure;
int16_t temp_MPU6050;

// MPU6050
#define MPU_ACCEL_XOUT1 0x3b
#define MPU_ACCEL_XOUT2 0x3c
#define MPU_ACCEL_YOUT1 0x3d
#define MPU_ACCEL_YOUT2 0x3e
#define MPU_ACCEL_ZOUT1 0x3f
#define MPU_ACCEL_ZOUT2 0x40

#define MPU_GYRO_XOUT1 0x43
#define MPU_GYRO_XOUT2 0x44
#define MPU_GYRO_YOUT1 0x45
#define MPU_GYRO_YOUT2 0x46
#define MPU_GYRO_ZOUT1 0x47
#define MPU_GYRO_ZOUT2 0x48

#define MPU_TEMP1 0x41
#define MPU_TEMP2 0x42

#define MPU_POWER1 0x6b
#define MPU_POWER2 0x6c


double acclX_scaled, acclY_scaled, acclZ_scaled;
double gyroX_scaled, gyroY_scaled, gyroZ_scaled;

void errExit(char* c)
{
	printf("%s\n", c);
	exit(-1);
}

void setGPIOPin(char* pin, char* dir, char* drive, char* val)
{
	char buf[256];
	int fd;

	// Open the GPIO Export file
	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1)
		errExit("GPIO Export");

	// Export chosen pin
	write(fd, pin, strlen(pin)); // Export GPIO pin
	close(fd);

	// Open exported pin DIRECTION file
	sprintf(buf, "/sys/class/gpio/gpio%s/direction", pin);
	fd = open(buf, O_WRONLY); // open GPIOxx direction file
	if (fd == -1)
		errExit("Gpio Direction");

	// write out the direction
	write(fd, dir, strlen(dir)); // set GPIOxx direction to out
	close(fd);

	// open the drive file
	sprintf(buf, "/sys/class/gpio/gpio%s/drive", pin);
	fd = open(buf, O_WRONLY); // open GPIO drive strength file
	if (fd == -1)
		errExit("Gpio Drive");

	write(fd, drive, strlen(drive)); // set GPIO drive
	close(fd);

	sprintf(buf, "/sys/class/gpio/gpio%s/value", pin);
	fd = open(buf, O_WRONLY);
	if (fd == -1)
		errExit("Gpio Value");

	write(fd, val, strlen(val)); // set GPIO value
	close(fd);

}
void DsetGPIOPin(char* pin, char* dir, char* drive, char* val)
{
	int fd;

	// Open the GPIO Export file
	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd == -1)
		errExit("GPIO Export");

	// Export chosen pin
	write(fd, pin, strlen(pin)); // Export GPIO pin
	close(fd);
}

// setMux - set up the multiplexor on A0 to connect it to ADC VIN0
void setMux(void)
{

	// Switch all the SPI1 pins through to the header pins. And enable level shifter.
	setGPIOPin("4", "out", "strong", "1"); // Level shifter OE
	setGPIOPin("29", "out", "strong", "0"); // SDA/SCL Mux
}
// DsetMux - set up the multiplexor on A0 to connect it to ADC VIN0
void DsetMux(void)
{

	// Switch all the SPI1 pins through to the header pins. And enable level shifter.
	DsetGPIOPin("4", "out", "strong", "1"); // Level shifter OE
	DsetGPIOPin("29", "out", "strong", "0"); // SDA/SCL Mux
}

// Set Mux GPIO for Leds
void setMuxLeds(void)
{

	// Left
	setGPIOPin("30", "out", "strong", "0"); // Arduino Pin3
	setGPIOPin("1", "out", "strong", "1");  // Arduino Pin3
	setGPIOPin("15", "out", "strong", "0");  // Arduino Pin3

	// right
	setGPIOPin("31", "out", "strong", "0"); // Arduino Pin2
	setGPIOPin("0", "out", "strong", "0");  // Arduino Pin2
	setGPIOPin("14", "out", "strong", "0");  // Arduino Pin2
}
// DSet Mux GPIO for Leds
void DsetMuxLeds(void)
{

	// Left
	DsetGPIOPin("30", "out", "strong", "0"); // Arduino Pin3
	DsetGPIOPin("1", "out", "strong", "1");  // Arduino Pin3
	DsetGPIOPin("15", "out", "strong", "0");  // Arduino Pin3

	// right
	DsetGPIOPin("31", "out", "strong", "0"); // Arduino Pin2
	DsetGPIOPin("0", "out", "strong", "0");  // Arduino Pin2
	DsetGPIOPin("14", "out", "strong", "0");  // Arduino Pin2
}

void setLeftLed(char *ch)
{
	setGPIOPin("15", "out", "strong", ch);  // Arduino Pin3
}
void setRightLed(char *ch)
{
	setGPIOPin("14", "out", "strong", ch);  // Arduino Pin2
}

double dist(double a, double b)
{
	return sqrt((a*a) + (b*b));
}

// Open a connection to the bmp085
// Returns a file id
int bmp085_i2c_Begin()
{
	int fd;
	char *fileName = "/dev/i2c-0";

	// Open port for reading and writing
	if ((fd = open(fileName, O_RDWR)) < 0)
		exit(1);

	// Set the port options and set the address of the device
	if (ioctl(fd, I2C_SLAVE, BMP085_I2C_ADDRESS) < 0) {
		close(fd);
		exit(1);
	}

	return fd;
}

// Read two words from the BMP085 and supply it as a 16 bit integer
__s32 bmp085_i2c_Read_Int(int fd, __u8 address)
{
	__s32 res = i2c_smbus_read_word_data(fd, address);
	if (res < 0) {
		close(fd);
		exit(1);
	}

	// Convert result to 16 bits and swap bytes
	res = ((res << 8) & 0xFF00) | ((res >> 8) & 0xFF);

	return res;
}

//Write a byte to the BMP085
void bmp085_i2c_Write_Byte(int fd, __u8 address, __u8 value)
{
	if (i2c_smbus_write_byte_data(fd, address, value) < 0) {
		close(fd);
		exit(1);
	}
}

// Read a block of data BMP085
void bmp085_i2c_Read_Block(int fd, __u8 address, __u8 length, __u8 *values)
{
	if (i2c_smbus_read_i2c_block_data(fd, address, length, values)<0) {
		close(fd);
		exit(1);
	}
}


void bmp085_Calibration()
{
	int fd;
	printf("Start read fd\n");
	fd = bmp085_i2c_Begin();
	printf("fd 1: %X2\n", fd);
	close(fd);
	fd = bmp085_i2c_Begin();
	printf("fd 2: %X2\n", fd);
	ac1 = bmp085_i2c_Read_Int(fd, 0xAA);
	ac2 = bmp085_i2c_Read_Int(fd, 0xAC);
	ac3 = bmp085_i2c_Read_Int(fd, 0xAE);
	ac4 = bmp085_i2c_Read_Int(fd, 0xB0);
	ac5 = bmp085_i2c_Read_Int(fd, 0xB2);
	ac6 = bmp085_i2c_Read_Int(fd, 0xB4);
	b1 = bmp085_i2c_Read_Int(fd, 0xB6);
	b2 = bmp085_i2c_Read_Int(fd, 0xB8);
	mb = bmp085_i2c_Read_Int(fd, 0xBA);
	mc = bmp085_i2c_Read_Int(fd, 0xBC);
	md = bmp085_i2c_Read_Int(fd, 0xBE);
	close(fd);
}

// Read the uncompensated temperature value
unsigned int bmp085_ReadUT()
{
	unsigned int ut = 0;
	int fd = bmp085_i2c_Begin();

	// Write 0x2E into Register 0xF4
	// This requests a temperature reading
	bmp085_i2c_Write_Byte(fd, 0xF4, 0x2E);

	// Wait at least 4.5ms
	usleep(5000);

	// Read the two byte result from address 0xF6
	ut = bmp085_i2c_Read_Int(fd, 0xF6);

	// Close the i2c file
	close(fd);

	return ut;
}

// Read the uncompensated pressure value
unsigned int bmp085_ReadUP()
{
	unsigned int up = 0;
	int fd = bmp085_i2c_Begin();

	// Write 0x34+(BMP085_OVERSAMPLING_SETTING<<6) into register 0xF4
	// Request a pressure reading w/ oversampling setting
	bmp085_i2c_Write_Byte(fd, 0xF4, 0x34 + (BMP085_OVERSAMPLING_SETTING << 6));

	// Wait for conversion, delay time dependent on oversampling setting
	usleep((2 + (3 << BMP085_OVERSAMPLING_SETTING)) * 1000);

	// Read the three byte result from 0xF6
	// 0xF6 = MSB, 0xF7 = LSB and 0xF8 = XLSB
	__u8 values[3];
	bmp085_i2c_Read_Block(fd, 0xF6, 3, values);

	up = (((unsigned int)values[0] << 16) | ((unsigned int)values[1] << 8) | (unsigned int)values[2]) >> (8 - BMP085_OVERSAMPLING_SETTING);

	// Close the i2c file
	close(fd);

	return up;
}

// Calculate pressure given uncalibrated pressure
// Value returned will be in units of Pa
unsigned int bmp085_GetPressure(unsigned int up)
{
	int x1, x2, x3, b3, b6, p;
	unsigned int b4, b7;

	b6 = b5 - 4000;
	// Calculate B3
	x1 = (b2 * (b6 * b6) >> 12) >> 11;
	x2 = (ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((int)ac1) * 4 + x3) << BMP085_OVERSAMPLING_SETTING) + 2) >> 2;

	// Calculate B4
	x1 = (ac3 * b6) >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (unsigned int)(x3 + 32768)) >> 15;

	b7 = ((unsigned int)(up - b3) * (50000 >> BMP085_OVERSAMPLING_SETTING));
	if (b7 < 0x80000000)
		p = (b7 << 1) / b4;
	else
		p = (b7 / b4) << 1;

	x1 = (p >> 8) * (p >> 8);
x1 = (x1 * 3038) >> 16;
x2 = (-7357 * p) >> 16;
p += (x1 + x2 + 3791) >> 4;

return p;
}

// Calculate temperature given uncalibrated temperature
// Value returned will be in units of 0.1 deg C
unsigned int bmp085_GetTemperature(unsigned int ut)
{
	int x1, x2;

	x1 = (((int)ut - (int)ac6)*(int)ac5) >> 15;
	x2 = ((int)mc << 11) / (x1 + md);
	b5 = x1 + x2;

	unsigned int result = ((b5 + 8) >> 4);

	return result;
}


// MPU6050 sensor
double get_z_rotation(double x, double y, double z)
{
	double radians;
	radians = atan2(z, dist(x, y));
	return -(radians * (180.0 / M_PI));
}

double get_y_rotation(double x, double y, double z)
{
	double radians;
	radians = atan2(x, dist(y, z));
	return -(radians * (180.0 / M_PI));
}

double get_x_rotation(double x, double y, double z)
{
	double radians;
	radians = atan2(y, dist(x, z));
	return (radians * (180.0 / M_PI));
}

int main(int argc, char **argv)
{
	int fd;
	char *fileName = "/dev/i2c-0";
	int  address = 0x68;
	char DebugDtata = argc;
	int16_t xaccel = 0;
	int16_t yaccel = 0;
	int16_t zaccel = 0;

	int16_t xgyro = 0;
	int16_t ygyro = 0;
	int16_t zgyro = 0;

	double x_rotation = 0;
	double y_rotation = 0;
	double z_rotation = 0;

	printf("Start MPU6050 \n");
	// DsetMux();
	// DsetMuxLeds();
	setMux();
	setMuxLeds();
	bmp085_Calibration();

	if ((fd = open(fileName, O_RDWR)) < 0) {
		printf("Failed to open i2c port\n");
		exit(1);
	}

	if (ioctl(fd, I2C_SLAVE, address) < 0) {
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}

	int8_t power = i2c_smbus_read_byte_data(fd, MPU_POWER1);
	i2c_smbus_write_byte_data(fd, MPU_POWER1, ~(1 << 6) & power);


	// First read
	// BMP085
	temperature = bmp085_GetTemperature(bmp085_ReadUT());
	pressure = bmp085_GetPressure(bmp085_ReadUP());
	printf("Temperature\t%0.1f*C\n", ((double)temperature) / 10);
    printf("Pressure\t%0.2fhPa\n", ((double)pressure) / 100);

	// MPU6050
	temp_MPU6050 += (i2c_smbus_read_byte_data(fd, MPU_TEMP1) << 8 | i2c_smbus_read_byte_data(fd, MPU_TEMP2));
	temp_MPU6050 /= 2;
	xaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT2);
	yaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT2);
	zaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT2);

	xgyro = i2c_smbus_read_byte_data(fd, MPU_GYRO_XOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_GYRO_XOUT2);
	ygyro = i2c_smbus_read_byte_data(fd, MPU_GYRO_YOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_GYRO_YOUT2);
	zgyro = i2c_smbus_read_byte_data(fd, MPU_GYRO_ZOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_GYRO_ZOUT2);

	printf("temp MPU6050: %0.1f*C\n", (float)temp_MPU6050 / 340.0f + (36.53 - 0.8));
	printf("Temp BMP085:  %0.1f*C\n", ((double)temperature) / 10);
	printf("Pressure:     %0.2fhPa\n", ((double)pressure) / 100);
	printf("accel x,y,z: %d, %d, %d\n", (int)xaccel, (int)yaccel, (int)zaccel);

	acclX_scaled = xaccel / 16384.0;
	acclY_scaled = yaccel / 16384.0;
	acclZ_scaled = zaccel / 16384.0;

	x_rotation = get_x_rotation(acclX_scaled, acclY_scaled, acclZ_scaled);
	y_rotation = get_y_rotation(acclX_scaled, acclY_scaled, acclZ_scaled);
	z_rotation = get_z_rotation(acclX_scaled, acclY_scaled, acclZ_scaled);

	printf("My X rotation: %f\n", x_rotation);
	printf("My Y rotation: %f\n", y_rotation);
	printf("My Z rotation: %f\n", z_rotation);
	if (x_rotation > 10.0)
	{
		setRightLed("1");
	}
	else
	{
		setRightLed("0");
	}

	if (x_rotation < -10.0)
	{
		setLeftLed("1");
	}
	else
	{
		setLeftLed("0");
	}
	printf("gyro x,y,z: %d, %d, %d\n", (int)xgyro, (int)ygyro, (int)zgyro);

	gyroX_scaled = xgyro / 131.0;
	gyroY_scaled = ygyro / 131.0;
	gyroZ_scaled = zgyro / 131.0;

	printf("My gyroX_scaled: %f\n", gyroX_scaled);
	printf("My gyroY_scaled: %f\n", gyroY_scaled);
	printf("My gyroZ_scaled: %f\n\n", gyroZ_scaled);

	while (1) {

		// BMP085
		temperature = bmp085_GetTemperature(bmp085_ReadUT());
		pressure = bmp085_GetPressure(bmp085_ReadUP());

		// printf("Temperature\t%0.1f*C\n", ((double)temperature) / 10);
		// printf("Pressure\t%0.2fhPa\n", ((double)pressure) / 100);

		// MPU6050
		temp_MPU6050 += (i2c_smbus_read_byte_data(fd, MPU_TEMP1) << 8 | i2c_smbus_read_byte_data(fd, MPU_TEMP2));
		temp_MPU6050 /= 2;
		xaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_ACCEL_XOUT2);
		yaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_ACCEL_YOUT2);
		zaccel = i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_ACCEL_ZOUT2);

		xgyro = i2c_smbus_read_byte_data(fd, MPU_GYRO_XOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_GYRO_XOUT2);
		ygyro = i2c_smbus_read_byte_data(fd, MPU_GYRO_YOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_GYRO_YOUT2);
		zgyro = i2c_smbus_read_byte_data(fd, MPU_GYRO_ZOUT1) << 8 | i2c_smbus_read_byte_data(fd, MPU_GYRO_ZOUT2);

		if (DebugDtata > 1)
		{
			// printf("DebugDtata: %d\n", DebugDtata);
			printf("temp MPU6050: %0.1f*C\n", (float)temp_MPU6050 / 340.0f + (36.53 - 0.8));
			printf("Temp BMP085:  %0.1f*C\n", ((double)temperature) / 10);
			printf("Pressure:     %0.2fhPa\n", ((double)pressure) / 100);
			
		}
		// printf("accel x,y,z: %d, %d, %d\n", (int)xaccel, (int)yaccel, (int)zaccel);
		acclX_scaled = xaccel / 16384.0;
		acclY_scaled = yaccel / 16384.0;
		acclZ_scaled = zaccel / 16384.0;

		x_rotation = get_x_rotation(acclX_scaled, acclY_scaled, acclZ_scaled);
		y_rotation = get_y_rotation(acclX_scaled, acclY_scaled, acclZ_scaled);
		z_rotation = get_z_rotation(acclX_scaled, acclY_scaled, acclZ_scaled);

		//printf("My X rotation: %f\n", x_rotation);
		//printf("My Y rotation: %f\n", y_rotation);
		//printf("My Z rotation: %f\n", z_rotation);
		if (x_rotation > 10.0)
		{
			setRightLed("1");
			printf("setRightLed(1)\n");
		}
		else
		{
			setRightLed("0");
		}

		if (x_rotation < -10.0)
		{			
			setLeftLed("1");
			printf("setLeftLed(1)\n");
		}
		else
		{
			setLeftLed("0");
		}
		//printf("gyro x,y,z: %d, %d, %d\n", (int)xgyro, (int)ygyro, (int)zgyro);

		gyroX_scaled = xgyro / 131.0;
		gyroY_scaled = ygyro / 131.0;
		gyroZ_scaled = zgyro / 131.0;

		//printf("My gyroX_scaled: %f\n", gyroX_scaled);
		//printf("My gyroY_scaled: %f\n", gyroY_scaled);
		//printf("My gyroZ_scaled: %f\n\n", gyroZ_scaled);

		sleep(1);
	}

	return 0;
}
