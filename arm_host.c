#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <arpa/inet.h>
#include "prussdrv.h"
#include "pruss_intc_mapping.h"

#define PRU_NUM 0			// define which pru is used
#define SHM_OFFSET 2048		// http://www.embedded-things.com/bbb/understanding-bbb-pru-shared-memory-access/

#define OUR_IP 0x04030201

#define SERVER 0
#define CLIENT 1

char conf_file_list[2][100] = {
	"server_ip.txt",
	"client_ip.txt"};


volatile sig_atomic_t sig_exit = 0;

volatile unsigned int* pruDataMem = NULL;

void handle_exit(int sig) 
{ 
	  sig_exit = 1; 
}

int pru_init(void)
{
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();
	if (prussdrv_open(PRU_EVTOUT_0))
	{
		return -1;
	}
	else
	{
		prussdrv_pruintc_init(&pruss_intc_initdata);
		return 0;
	}
}

void pru_load(int pru_num, char* datafile, char* codefile)
{
	/* load datafile in PRU memory */
	prussdrv_load_datafile(pru_num, datafile);
	/* load and execute codefile in PRU */
	prussdrv_exec_program(pru_num, codefile);
}

void pru_stop(int pru_num)
{
	prussdrv_pru_disable(pru_num);
	prussdrv_exit();
}

volatile int32_t* init_prumem()
{
	volatile int32_t* p;
	prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, (void**)&p);
	return p+SHM_OFFSET;
}

int get_ip_conf(int ip_index)
{
	FILE* f = fopen(conf_file_list[ip_index], "r");
	uint8_t buf[20];
	uint32_t ip;
	
	if(f == NULL)
	{
		printf("error: cannot open file\n");
		return -1;
	}
	
	if(0 > fscanf(f, "%s", buf))
	{
		printf("error: read file");
		return -1;
	}
	
	if(1 != inet_pton(AF_INET, buf, &ip))
	{
		printf("error: ip format not valid\n");
		return -1;
	}

	pruDataMem[0] = ip;
	pruDataMem[1] = 1;

	sleep(1);

	if(pruDataMem[1] != ip) 
	{
		printf("error: no response from PRU %x\n", pruDataMem[1]);
		return -1;
	}
	else
	{
		printf("ip configured: %d.%d.%d.%d\n", ip & 0xff, (ip >> 8) & 0xff, (ip >> 16) & 0xff, (ip >> 24) & 0xff);
	}

	return 0;
}

int main(void)
{
	signal(SIGINT, handle_exit); 
	/* initialize the PRU */
	printf("pruss driver init (%i)\n", pru_init());

	/* load the PRU code (consisting of both code and data bin file). */
	pru_load(PRU_NUM, "pru_data.bin", "pru_code.bin");

	/* get the memory pointer to the shared data segment */
	pruDataMem = init_prumem();

	/* read erver ip and send to pru */
	printf("configuring server ip\n");
	if(get_ip_conf(SERVER))	
	{
		printf("stopping  PRU\n");
		pru_stop(PRU_NUM);
		return 1;
	}

	/* read client ip and send to pru */
	printf("configuring client ip\n");
	if(get_ip_conf(CLIENT))	
	{
		printf("stopping  PRU\n");
		pru_stop(PRU_NUM);
		return 1;
	}

	while(1)
	{
		if(sig_exit) break;
		//update_icmp_stats();
	}

	printf("stopping  PRU\n");
	pru_stop(PRU_NUM);

	return 0;
}
