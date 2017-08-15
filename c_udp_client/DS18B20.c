#include "common.h"

int Temperature_DS18B20 = 0;
pthread_t DS18B20_thread; // DS18B20 thread
volatile int thread_ds18b20_arg = 0; // nothing doing, only for disapp. warning

void* do_DS18B20_thread(void *arg)
{
	if (arg == NULL){
		thread_ds18b20_arg = 0;
	}
	else{
		thread_ds18b20_arg = 1; 
	}
	pthread_t id = pthread_self();
	while (1){
		if (pthread_equal(id, DS18B20_thread))
		{
			// here function for read temperature
			printf("Test Thread DS18B20\t%0.1f*C\n", ((double)Temperature_DS18B20) / 10);
		}
		else
		{
			printf("\n Error\n");
		}
		// usleep(100000); // 100mSec
		sleep(1);
	}
	return NULL;
}




