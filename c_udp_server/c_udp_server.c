/* Sample UDP server */

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <strings.h>
#include <signal.h>
#include "timer.h"
int var = 0;

void timer_handler(void);

void timer_handler(void)
{
	printf("timer: var is %i\n", var++);
}

int main(int argc, char**argv)
{
	int sockfd, n;
	struct sockaddr_in servaddr, cliaddr;
	socklen_t len;
	char mesg[1000];
	const char stop_msg[] = "stop\n";
    int addr = 32000;
	
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	bzero(&servaddr, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servaddr.sin_port = htons(addr);
	bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
	printf("Start Server, addr: %d \n", addr);

	if (start_timer(1000, &timer_handler))
	{
		printf("\n timer error\n");
		return(1);
	}

	printf("\npress ctl-c to quit.\n");

	while (1)
	{
		if (var > 5)
		{
			break;
		}
	}
	for (;;)
	{
		len = sizeof(cliaddr);
		n = recvfrom(sockfd, mesg, 1000, 0, (struct sockaddr *)&cliaddr, &len);
		if (n > 0){
			printf("Lenght: %d \n", n);
			
			sendto(sockfd, mesg, n, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
			n = sizeof(addr);
			sendto(sockfd, stop_msg, n, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
			printf("-------------------------------------------------------\n");
			mesg[n] = 0;
			printf("Received the following:\n");

			printf("%s", mesg);
			printf("-------------------------------------------------------\n");
		}
	}

	stop_timer();
	return(0);
}
