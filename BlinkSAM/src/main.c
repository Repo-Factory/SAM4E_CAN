/* 
 * @ Zix
 * SAM4E CAN Library
 */

#include "asf.h"
#include "can.h"

#define CAN1_PIO        	   PIOC
#define CAN1_RX_PIN     	   PIO_PC12
#define CAN1_TX_PIN     	   PIO_PC15
#define CAN1_IRQ	    	   CAN1_IRQn
#define CAN1_BASE       	   CAN1
#define CAN1_PID        	   ID_CAN1
volatile Can* canController = (volatile Can*)CAN1_BASE;

#define LED0_PIO    		   PIOD
#define LED0_PIN    		   PIO_PD22
#define SET_LIGHT_ON()         LED0_PIO->PIO_CODR = LED0_PIN
#define SET_LIGHT_OFF()        LED0_PIO->PIO_SODR = LED0_PIN
#define LIGHT_ON_SUB_ID        0b00000001111
#define LIGHT_OFF_SUB_ID       0b00000000001

#define LIGHT_MB 	   	0
#define LIGHT_MB_MASK  	0b00000001111
#define LIGHT_MB_START	0b00000000001

void CAN1_Handler(void)
{
	const uint32_t id_received = canController->CAN_MB[LIGHT_MB].CAN_MID;
	switch (id_received)
	{
		case LIGHT_ON_SUB_ID:
			SET_LIGHT_ON();
			break;
		case LIGHT_OFF_SUB_ID:
			SET_LIGHT_OFF();
			break;
	}
}

static void hardware_init(void)
{
	/* 
	 * This simply configures LED as output pin and clears it
	 */
	LED0_PIO->PIO_OER = LED0_PIN;
    LED0_PIO->PIO_CODR = LED0_PIN;
	
	/* 
	 *	The CAN controller clock must be activated by the Power Management Controller (PMC) 
	 *	and the CAN controller interrupt line must be enabled by the interrupt controller before use
	 */
	PMC->PMC_PCER1 |= PMC_PCER1_PID38; // Peripheral Identifier of CAN is 38, bit 6 of peripheral clock enable register corresponds to PID38 
	NVIC_EnableIRQ(CAN1_IRQ);

	/* 
	 * Set I/O lines to be controlled by CAN peripheral instead of PIO Controller
	 * We'll assign CAN as peripheral A on TX/RX pins by settings ABCD1 and ABCD2 to 0 for
	 * corresponding pins.
	 * Disable PIO Controller on TX and RX pins at PIO Disable Register
	 */
	CAN1_PIO->PIO_ABCDSR[0] &= ~(CAN1_RX_PIN | CAN1_TX_PIN);
	CAN1_PIO->PIO_ABCDSR[1] &= ~(CAN1_RX_PIN | CAN1_TX_PIN);
	CAN1_PIO->PIO_PDR        =  (CAN1_RX_PIN | CAN1_TX_PIN);
	
	
	/* 
	 * Enable CAN Controller and interrupts on Mailbox 0
	 */
	canController->CAN_MR |= CAN_MR_CANEN;
	canController->CAN_MR &= ~CAN_MR_TTM; // Timestamping Mode is enabled by clearing the TTM bit in the CAN_MR
	canController->CAN_IER |= CAN_IER_MB0;

	/* 
	 * Mailbox 0 will accept messages based on an acceptance Mask, this will have to be set
	 * by us based on our ID layout. Configure the mailbox as a receive box.
	 */
	canController->CAN_MB[LIGHT_MB].CAN_MAM = LIGHT_MB_MASK;
	canController->CAN_MB[LIGHT_MB].CAN_MID = LIGHT_MB_START; // Default, will change on message acceptance
	canController->CAN_MB[LIGHT_MB].CAN_MMR = CAN_MMR_MOT_MB_RX_OVERWRITE;
}

int main(void)
{
  	hardware_init();
}















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