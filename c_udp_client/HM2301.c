#include "common.h"


int Temperature_HM2301 = 0;
int Humidity_HM2301 = 0;
pthread_t HM2301_thread; // DS18B20 thread
volatile int thread_HM2301_arg = 0; // nothing doing, only for disapp. warning

void* do_HM2301_thread(void *arg)
{
	if (arg == NULL){
		thread_HM2301_arg = 0;
	}
	else{
		thread_HM2301_arg = 1;
	}
	pthread_t id = pthread_self();
	while (1){
		if (pthread_equal(id, HM2301_thread))
		{
			// here function for read temperature
			printf("Test Thread HM2301\t%0.1f*C\n", ((double)Temperature_HM2301) / 10);
			printf("Test Thread HM2301\t%0.1f*C\n", ((double)Humidity_HM2301) / 10);
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




