/* 
 * @ Zix
 * SAM4E CAN Example From ASF Page https://asf.microchip.com/docs/latest/sam4e/html/sam_can_quickstart.html
 */

#include "asf.h"
#include "can_asf.h"

#define CAN1_PIO        	   PIOC
#define CAN1_RX_PIN     	   PIO_PC12
#define CAN1_TX_PIN     	   PIO_PC15
#define CAN1_IRQ	    	   CAN1_IRQn
#define CAN1_BASE       	   CAN1
#define CAN1_PID        	   ID_CAN1

#define LED0_PIO    		   PIOD
#define LED0_PIN    		   PIO_PD22
#define SET_LIGHT_ON()         LED0_PIO->PIO_CODR = LED0_PIN
#define SET_LIGHT_OFF()        LED0_PIO->PIO_SODR = LED0_PIN
#define LIGHT_ON_SUB_ID        0b00000001111
#define LIGHT_OFF_SUB_ID       0b00000000001

volatile Can* canController = (volatile Can*)CAN1_BASE;

int main(void)
{
	const unsigned long ul_sysclk = SystemCoreClock;
	can_mb_conf_t can0_mailbox;
	can_mb_conf_t can1_mailbox;
	pmc_enable_periph_clk(ID_CAN1);
	can_init(CAN1, ul_sysclk, CAN_BPS_500K);
	can_reset_all_mailbox(CAN1);
	can1_mailbox.ul_mb_idx = 0;
	can1_mailbox.uc_obj_type = CAN_MB_RX_MODE;
	can1_mailbox.ul_id_msk = CAN_MAM_MIDvA_Msk | CAN_MAM_MIDvB_Msk;
	can1_mailbox.ul_id = CAN_MID_MIDvA(0x07);
	can_mailbox_init(CAN1, &can1_mailbox);
	can0_mailbox.ul_mb_idx = 0;
	can0_mailbox.uc_obj_type = CAN_MB_TX_MODE;
	can0_mailbox.uc_tx_prio = 15;
	can0_mailbox.uc_id_ver = 0;
	can0_mailbox.ul_id_msk = 0;
	can_mailbox_init(CAN1, &can0_mailbox);
	can0_mailbox.ul_id = CAN_MID_MIDvA(0x07);
	can0_mailbox.ul_datal = 0x12345678;
	can0_mailbox.ul_datah = 0x87654321;
	can0_mailbox.uc_length = 8;
	can_mailbox_write(CAN1, &can0_mailbox);

	for (;;)
	{
		// can_global_send_transfer_cmd(CAN0, CAN_TCR_MB0);
		while (!(can_mailbox_get_status(CAN1, 0) & CAN_MSR_MRDY)) {
		}
		can_mailbox_read(CAN1, &can1_mailbox);
		const int data_rcvd = can1_mailbox.ul_datal;
		switch (data_rcvd)
		{
			case LIGHT_ON_SUB_ID:
				SET_LIGHT_ON();
				break;
			case LIGHT_OFF_SUB_ID:
				SET_LIGHT_OFF();
				break;
		}
	}
}


/* SAM4E Series [DATASHEET]
Atmel-11157H-ATARM-SAM4E16-SAM4E8-Datasheet_31-Mar-16
652
Example of bit timing determination for CAN baudrate of 500 kbit/s:
fPeripheral clock = 48 MHz
CAN baudrate = 500 kbit/s => bit time = 2 μs
Delay of the bus driver: 50 ns
Delay of the receiver: 30 ns
Delay of the bus line (20 m): 110 ns */


/* After power-up reset, the CAN controller is disabled. The CAN controller clock must be activated by the Power
Management Controller (PMC) and the CAN controller interrupt line must be enabled by the interrupt controller.
The CAN controller must be initialized with the CAN network parameters. The CAN_BR defines the sampling point
in the bit time period. CAN_BR must be set before the CAN controller is enabled.
The CAN controller is enabled by setting the CANEN bit in the CAN_MR. At this stage, the internal CAN controller
state machine is reset, error counters are reset to 0, and error flags are reset to 0.
Once the CAN controller is enabled, bus synchronization is done automatically by scanning eleven recessive bits.
The WAKEUP bit in the CAN_SR is automatically set to 1 when the CAN controller is synchronized (WAKEUP and
SLEEP are stuck at 0 after a reset).
The CAN controller can start listening to the network in Autobaud Mode. In this case, the error counters are locked
and a mailbox may be configured in Receive Mode. By scanning error flags, the CAN_BR values synchronized
with the network. Once no error has been detected, the application disables the Autobaud Mode, clearing the ABM
bit in the CAN_MR. */




/* 
enum 
	CONTROLLER
	SYSTEM_MONITOR
	POWER_SYSTEM
	MOTION_CONTROL
	RESERVED
 */









/* 

void (*mail_handle[RX_MB_NUM])(void) = 
{
	&HANDLE_CONTROLLER,
	&HANDLE_SYSTEM_MONITOR,
	&HANDLE_POWER_SYSTEM,
	&HANDLE_MOTION_CONTROL,
	&HANDLE_RESERVED,
	&HANDLE_AUXILLARY
};
	void transmit_msg(void)
	{

	}


void HANDLE_CONTROLLER(void)
{

}

void HANDLE_SYSTEM_MONITOR(void)
{

}

void HANDLE_POWER_SYSTEM(void)
{

}

void HANDLE_MOTION_CONTROL(void)
{

}

 | CAN_IER_MB1 |
				  CAN_IER_MB2 | CAN_IER_MB3 |
				  CAN_IER_MB4 | CAN_IER_MB5 |
				  CAN_IER_MB6 | CAN_IER_MB7;

void HANDLE_RESERVED(void)
{

}
CAN1_PIO->PIO_ODR = CAN1_RX_PIN;
	CAN1_PIO->PIO_OER = CAN1_TX_PIN;

	CAN_PIO->PIO_ABCDSR[0] &= ~(CAN_RX_PIN | CAN_TX_PIN) ;
	CAN_PIO->PIO_ABCDSR[1] &= ~(CAN_RX_PIN | CAN_TX_PIN) ;
	CAN_PIO->PIO_PDR        =  (CAN_RX_PIN | CAN_TX_PIN) ;



	uint32_t populate_can_frame(const Frame_Format format)
	{
		
	}

		const uint64_t data = (CAN_MDH[AUXILLARY] << 32) |= CAN_MDL[AUXILLARY];


	uint32_t status = canController->CAN_IMR;
	canController->CAN_MB[transmit0_mb].CAN_MMR = CAN_MMR_MOT_MB_TX;

	#define BAUD_RATE 1000
#define RCV_FILTER 0x00
#define SEND_TIMES 1
#define SEND_INTERVAL 0
#define RX_MB_NUM 6

#define CONTROLLER_MB_MASK 00000001111b
#define SYSTEM_MONITOR_MB_MASK 00000011110b
#define POWER_SYSTEM_MB_MASK 00000111100b
#define MOTION_CONTROL_MB_MASK 00001111000b
#define RESERVED_MB_MASK 00011110000b
#define LIGHT_MB_MASK 00111100000b
#define TRANSMIT0_MB_MASK 01111000000b
#define TRANSMIT1_MB_MASK 11110000000b

#define LIGHT_MB_START 00111100000b

#define CONTROLLER_MB 0
#define SYSTEM_MONITOR_MB 1
#define POWER_SYSTEM_MB 2
#define MOTION_CONTROL_MB 3
#define RESERVED_MB 4
#define LIGHT_MB 5
#define TRANSMIT0_MB 6
#define TRANSMIT1_MB 7
 */