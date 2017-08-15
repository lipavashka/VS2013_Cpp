/* Sample UDP server */

//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
//#include <stdio.h>
//#include <strings.h>
//#include <string.h>
//#include <stdlib.h>
//#include <signal.h>
//#include "timer.h"


/*#include <stdio.h> 
#include <stdlib.h>
#include <pthread.h>*/ 
#include "common.h"
// #include "dht.h"

int var = 0;
int sockfd, n;
struct sockaddr_in servaddr, cliaddr;
char sendline[1000]; 
char recvline[1000];
int addr = 32000;
extern unsigned int temperature, pressure, Temperature_DS18B20, Humidity_HM2301, Temperature_HM2301;
// threads
extern pthread_t bmp085_thread; // bmp085 thread 
extern void* do_bmp085_thread(void *arg);
extern pthread_t DS18B20_thread; // DS18B20 thread
extern void* do_DS18B20_thread(void *arg);
extern pthread_t HM2301_thread; // HM2301 thread
extern void* do_HM2301_thread(void *arg);

// DHT21 (AM2301)
#define MAX_RETRIES 10
extern uint8_t data_pin = 0;
uint8_t power_pin = 0;
DHT_MODEL_t model = AUTO_DETECT;
int retry = MAX_RETRIES;

extern void dhtSetup(uint8_t power_pin, uint8_t pin, DHT_MODEL_t model);

void timer_handler(void);
void* doSomeThing(void *arg);

// extern uint8_t spi_static_TX_data[300];



void timer_handler(void)
{
	static char str[10];

	/*temperature = bmp085_GetTemperature(bmp085_ReadUT());
	pressure = bmp085_GetPressure(bmp085_ReadUP());
	printf("Temperature\t%0.1f*C\n", ((double)temperature) / 10);
	printf("Pressure\t%0.2fhPa\n", ((double)pressure) / 100);*/
	var++;
	printf("timer: var is %i\n", var);
	// sprintf(str, "%d", var);
	sprintf(str, "%d", temperature);
	sendto(sockfd, str, strlen(str), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
	sprintf(str, "%d", pressure);
	sendto(sockfd, str, strlen(str), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
}

int main(int argc, char**argv)
{
	int err;
	// test bmp085 sensor
	printf("Start \n");  
	setMux();
	// printf("bmp085_Calibration \n");
	bmp085_Calibration();
	// printf("bmp085_GetTemperature \n");
	temperature = bmp085_GetTemperature(bmp085_ReadUT()); 
	// printf("bmp085_GetPressure \n");
	pressure = bmp085_GetPressure(bmp085_ReadUP());

	// printf("Temperature\t%0.2f%cC\n", ((double)temperature) / 10, 0x00F8);
	printf("Temperature\t%0.1f*C\n", ((double)temperature) / 10);
	printf("Pressure\t%0.2fhPa\n", ((double)pressure) / 100);

	// Init DHT21 (AM2301)
	model = DHT22;
	data_pin = 0; // 25 - for GPIO
	power_pin = 0; // 38 - for GPIO
	// Init GPIO SPI
	
	printf("Init DHT21 (AM2301)\n");  
	/* Power on the sensor */
	if (power_pin)
	{
		// ubexport Gpio
//		unExport_GPIOPin("25");
//		unExport_GPIOPin("38");
		// Switch all the SPI1 pins through to the header pins. And enable level shifter.
		printf("DHT21 (AM2301) using GPIO Mode\n");
		setGPIOPin("4", "out", "strong", "1");  // Level shifter OE
		setGPIOPin("42", "out", "strong", "0"); // SPI1_CS (Pin_10)
		setGPIOPin("25", "out", "strong", "1"); // SPI1_MOSI (Pin_11 GPIO25)
		setGPIOPin("38", "out", "strong", "1"); // SPI1_MISO (Pin_12 GPIO38)
		setGPIOPin("55", "out", "strong", "0"); // SPI1_SCK (Pin_13)
		// setGPIOPin("25", "in", "pulldown", "0"); // data_pin
		/*dhtPoweron(power_pin);
		if (getStatus() != ERROR_NONE)
		{
			printf("Error during setup: %s\n", getStatusString());
			return -1;
		}
		else 
		{
			printf("Init void dhtPoweron(uint8_t pin) : Ok\n");
		}*/
	}
	else
	{
		// Switch all the SPI1 pins through to the header pins. And enable level shifter.
		//printf("DHT21 (AM2301) using SPI Mode\n");
		//setGPIOPin("4", "out", "strong", "1");  // Level shifter OE
		//setGPIOPin("42", "out", "strong", "0"); // SPI1_CS (Pin_10)
		//setGPIOPin("25", "out", "strong", "0"); // SPI1_MOSI (Pin_11 GPIO25) // 43
		//setGPIOPin("38", "out", "strong", "0"); // SPI1_MISO (Pin_12 GPIO38) // 54
		//setGPIOPin("55", "out", "strong", "0"); // SPI1_SCK (Pin_13)
	}
    

	///* Init sensor communication */
	dhtSetup(power_pin, data_pin, model);
	//printf("\n Main REQUEST");
	//int i = 0;
	//for (i = 0; i < 300; i++)
	//{
	//	// if((i%100)==0) printf("\n%03d ", i);
	//	if ((i % 100) == 0)
	//	{
	//		printf("\n%03d ", i);
	//	}
	//	if ((i % 40) == 0)
	//	{
	//		printf("\n");
	//	}
	//	printf("%02X", spi_static_TX_data[i]);
	//}
	if (getStatus() != ERROR_NONE)
	{
		printf("Error during setup: %s\n", getStatusString());
		return -1;
	}



	// Read sensor with retry 
	do
	{
		readSensor();

		if (getStatus() == ERROR_NONE)
		{
			printf("Rel. Humidity: %3.1f %%\n", getHumidity());
			printf("Temperature:   %3.1f *C\n", getTemperature()); // °C
		}
		else
		{
			printf("Error red sensor (DHT21 (AM2301))\n");
			sleep(1);
		}
	}
	while ((getStatus() != ERROR_NONE) && retry--);

	if (getStatus() != ERROR_NONE)
	{
		printf("Error reading sensor: %s\n", getStatusString());
	}








	if (argc != 2)
	{
		printf("usage:  udpcli <IP address>\n");
		exit(1);
	}
	

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	bzero(&servaddr, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(argv[1]);
	servaddr.sin_port = htons(32000);
	printf("Start Client, addr: %d \n", addr);

	if (start_timer(1000, &timer_handler)) // 2 mSec
	{
		printf("\n timer error\n");

		return(1);
	}

	printf("\npress ctl-c to quit.\n");

	// err = pthread_create(&(bmp085_thread), NULL, &do_bmp085_thread, NULL);
	err = pthread_create(&bmp085_thread, NULL, do_bmp085_thread, NULL);
	if (err != 0)
		printf("\ncan't create bmp085_thread :[%s]", strerror(err));
	else
		printf("\n bmp085_thread created successfully\n");

	err = pthread_create(&DS18B20_thread, NULL, do_DS18B20_thread, NULL);
	if (err != 0)
		printf("\ncan't create DS18B20_thread :[%s]", strerror(err));
	else
		printf("\n DS18B20_thread created successfully\n");

	err = pthread_create(&HM2301_thread, NULL, do_HM2301_thread, NULL);
	if (err != 0)
		printf("\ncan't create HM2301_thread :[%s]", strerror(err));
	else
		printf("\n HM2301_thread created successfully\n");

	while (1)
	{
		if (fgets(sendline, 10000, stdin) != NULL){
			sendto(sockfd, sendline, strlen(sendline), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
			n = recvfrom(sockfd, recvline, 10000, 0, NULL, NULL);
			recvline[n] = 0;
			fputs(recvline, stdout);
		}
	}

	stop_timer();
	return(0);
}
