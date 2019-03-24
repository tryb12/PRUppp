// From: http://git.ti.com/pru-software-support-package/pru-software-support-package/trees/master/pru_cape/pru_fw/PRU_Hardware_UART

#include <stdint.h>
#include <pru_uart.h>
#include "resource_table_empty.h"
#include "PRUppp.h"

/* The FIFO size on the PRU UART is 16 bytes; however, we are (arbitrarily)
 * only going to send 8 at a time */
#define FIFO_SIZE	16
#define MAX_CHARS	8
#define BUFFER		40

uint8_t rx_buffer[BUFFER];
uint16_t rx_buf_idx = 0;
uint8_t frame_start = 0;
uint8_t frame_end = 0;

uint8_t tx_buffer[BUFFER];
uint16_t tx_buf_idx = 0;

void Rx(void)
{
	while (CT_UART.LSR_bit.DR)
	{	
		uint8_t rx;
		rx = CT_UART.RBR_bit.DATA;
		if(rx == 0x7e)
		{
			if(!frame_start)
			{
				frame_start = 1;	
			}
			else
			{
				frame_end = 1;	
				break;
			}
		}
		else
		{	
			if(frame_start && !frame_end) 
			{
				rx_buffer[rx_buf_idx++] = rx;
			}
		}

		if(rx_buf_idx == BUFFER)
		{
			rx_buf_idx = 0;
			frame_start = 0;
			frame_end = 0;
		}
	}
}

void Tx(void)
{
	uint16_t cnt = 0;
	/* Check if the TX FIFO and the TX SR are completely empty */
	if (CT_UART.LSR_bit.TEMT)
	{
		while (tx_buffer[tx_buf_idx] != NULL && cnt < MAX_CHARS) 
		{
			CT_UART.THR = tx_buffer[tx_buf_idx];
			tx_buf_idx++;
			cnt++;
		}
	}
}


#define CONFIGURE_REQ 		0x01
#define CONFIGURE_ACK 		0x02
#define CONFIGURE_NAK 		0x03
#define CONFIGURE_REJECT	0x04
#define TERMINATE_REQ 		0x05
#define TERMINATE_ACK 		0x06
#define ECHO_REQ 		0x09
#define ECHO_REPLY 		0x0A

void processLCP(cpFrame* lcp)
{
	switch(lcp->code)
	{
	case LCP_CONFIGURE_REQ:
		
		break;
	case LCP_CONFIGURE_ACK: 
		
		break;
	case LCP_ECHO_REQ:

		break;
	case LCP_ECHO_REPLY:
		break;
	default:
		break;
	}

}

handleIPCPConfigReq()
{

}

void processNCP(cpFrame* ncp)
{
	switch(ncp->code)
	{
	case LCP_CONFIGURE_REQ:
		handleIPCPConfigReq(ncp->data, ncp->length);	
		break;
	case LCP_CONFIGURE_ACK: 
		break;
	case LCP_CONFIGURE_NAK: 
		
		break;
	case LCP_ECHO_REQ:

		break;
	case LCP_ECHO_REPLY:
		break;
	default:
		break;
	}


}

void processIP(ipv4Header* ip)
{

}

void Process(void)
{
	uint16_t idx = 0;
	if(frame_start && frame_end)
	{
		pppHeader * ppp = (pppHeader*)&rx_buffer[0];
		
		switch(ppp->protocol)
		{
		case LCP:
			processLCP((cpFrame*)ppp->data);
			break;
		case NCP:
			processNCP((cpFrame*)ppp->data);
			break;
		case IP:
			processIP((ipv4Header*)ppp->data);
			break;
		default:
			break;
		}

		tx_buf_idx = 0;
		rx_buf_idx = 0;
		frame_start = frame_end = 0;
	}
}

void init(void)
{
	/*** INITIALIZATION ***/

	/* Set up UART to function at 115200 baud - DLL divisor is 104 at 16x oversample
	 * 192MHz / 104 / 16 = ~115200 */
	CT_UART.DLL = 104;
	CT_UART.DLH = 0;
	CT_UART.MDR_bit.OSM_SEL = 0x0;

	/* Enable Interrupts in UART module. This allows the main thread to poll for
	 * Receive Data Available and Transmit Holding Register Empty */
	CT_UART.IER = 0x7;

	/* If FIFOs are to be used, select desired trigger level and enable
	 * FIFOs by writing to FCR. FIFOEN bit in FCR must be set first before
	 * other bits are configured */
	/* Enable FIFOs for now at 1-byte, and flush them */
	CT_UART.FCR = (0x80) | (0x8) | (0x4) | (0x2) | (0x01); // 8-byte RX FIFO trigger

	/* Choose desired protocol settings by writing to LCR */
	/* 8-bit word, 1 stop bit, no parity, no break control and no divisor latch */
	CT_UART.LCR = 3;

	/* If flow control is desired write appropriate values to MCR. */
	/* No flow control for now, but enable loopback for test */
	CT_UART.MCR = 0x00;

	/* Choose desired response to emulation suspend events by configuring
	 * FREE bit and enable UART by setting UTRST and URRST in PWREMU_MGMT */
	/* Allow UART to run free, enable UART TX/RX */
	CT_UART.PWREMU_MGMT_bit.FREE = 0x1;
	CT_UART.PWREMU_MGMT_bit.URRST = 0x1;
	CT_UART.PWREMU_MGMT_bit.UTRST = 0x1;

	/* Turn off RTS and CTS functionality */
	CT_UART.MCR_bit.AFE = 0x0;
	CT_UART.MCR_bit.RTS = 0x0;

	/*** END INITIALIZATION ***/

}

void main(void)
{
	volatile uint32_t not_done = 1;

	init();

	while(1) {
		Rx();
		Process();
		Tx();	
	}

	/*** DONE SENDING DATA ***/
	/* Disable UART before halting */
	CT_UART.PWREMU_MGMT = 0x0;

	/* Halt PRU core */
	__halt();
}
