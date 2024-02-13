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
#define TOGGLE(BIT) 		   BIT^=1
#define LIGHT_ON_SUB_ID        0b00000001111
#define LIGHT_OFF_SUB_ID       0b00000000001
#define DISABLE_WR_PROTECT_CAN 0x0043414E
#define DISABLE_WR_PROTECT_PMC 0x00504D43
#define BAUD_RATE			   0x00053255
#define TOGGLE_PERIOD 		   100

#define LIGHT_MB 	   	0
#define LIGHT_MB_MASK  	0b00000001111
#define LIGHT_MB_START	0b00000000001

static uint32_t tickcount = 0;
static uint32_t light_on = 0;

void CAN1_Handler(void)
{
	// const uint32_t id_received = canController->CAN_MB[LIGHT_MB].CAN_MID;
	// switch (id_received)
	// {
	// 	case LIGHT_ON_SUB_ID:
	// 		SET_LIGHT_ON();
	// 		break;
	// 	case LIGHT_OFF_SUB_ID:
	// 		SET_LIGHT_OFF();
	// 		break;
	// }
}

void transmit_can_message(void)
{
	canController->CAN_MB[1].CAN_MDL = 0x01;
	canController->CAN_MB[1].CAN_MDH = 0x00;
	canController->CAN_TCR |= 1u << 1;
	// canController->CAN_MB[1].CAN_MCR = 0x00;
}

static void toggle_light(void)
{
	if (light_on)
		SET_LIGHT_OFF();
	else
		SET_LIGHT_ON();
	TOGGLE(light_on);
}

// Interrupt Routine (ISR) called in Response to SysTick Interrupt Request (IQR)
void SysTick_Handler(void)
{
	// if (++tickcount==100)
	// {
	// 	transmit_can_message();
	// 	toggle_light();
	// 	tickcount = 0;
	// }
}
static void hardware_init(void)
{
	/* 
	 * This simply configures LED as output pin and clears it
	 */
	LED0_PIO->PIO_OER = LED0_PIN;
    LED0_PIO->PIO_CODR = LED0_PIN;

	// SysTick_Config(SystemCoreClock/TOGGLE_PERIOD);
    // NVIC_EnableIRQ(SysTick_IRQn);
	SET_LIGHT_OFF();
	
	/* 
	 *	The CAN controller clock must be activated by the Power Management Controller (PMC) 
	 *	and the CAN controller interrupt line must be enabled by the interrupt controller before use
	 */
	//  Set CAN_BR register
	// This register can only be written if the WPEN bit is cleared in the CAN Write Protection Mode Registe

	canController->CAN_WPMR = DISABLE_WR_PROTECT_CAN; 
	PMC->PMC_WPMR  =  DISABLE_WR_PROTECT_PMC;
	canController->CAN_BR = BAUD_RATE;
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
	CAN1_PIO->PIO_PDR       |=  (CAN1_RX_PIN | CAN1_TX_PIN);
	
	
	// /* 
	//  * Enable CAN Controller and interrupts on Mailbox 0
	//  */
	// canController->CAN_MR |= CAN_MR_CANEN;
	// canController->CAN_MR &= ~CAN_MR_TTM; // Timestamping Mode is enabled by clearing the TTM bit in the CAN_MR

	// /* 
	//  * Mailbox 0 will accept messages based on an acceptance Mask, this will have to be set
	//  * by us based on our ID layout. Configure the mailbox as a receive box.
	//  */
	canController->CAN_IER |= CAN_IER_MB0;
	canController->CAN_MB[LIGHT_MB].CAN_MAM = LIGHT_MB_MASK;
	canController->CAN_MB[LIGHT_MB].CAN_MID = LIGHT_MB_START; // Default, will change on message acceptance
	canController->CAN_MB[LIGHT_MB].CAN_MMR = CAN_MMR_MOT_MB_RX_OVERWRITE;

	canController->CAN_MB[1].CAN_MMR = CAN_MMR_MOT_MB_TX;
	canController->CAN_IER |= CAN_IER_MB1;
}

int main(void)
{
  	hardware_init();
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