/* 
 * @ Zix
 * SAM4E CAN Library
 */

#include "asf.h"
#include "can_asf.h"

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
#define TOGGLE_PERIOD 		   100

#define DISABLE_WR_PROTECT_CAN 0x0043414E
#define DISABLE_WR_PROTECT_PMC 0x00504D43
#define BAUD_RATE			   0x00053255

#define LIGHT_MB 	   		   0
#define LIGHT_MB_MASK  		   0b00000001111
#define LIGHT_MB_START		   0b00000000001

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

void transmit_can_message(void)
{
	canController->CAN_MB[1].CAN_MDL = 0x01;
	canController->CAN_MB[1].CAN_MDH = 0x00;
	canController->CAN_MB[1].CAN_MCR = 0x00;
	canController->CAN_TCR |= 1u << 1;
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

	/* 
	 * Setup Transmission Mailbox
	 */
	canController->CAN_MB[1].CAN_MMR = CAN_MMR_MOT_MB_TX;
	canController->CAN_IER |= CAN_IER_MB1;
}

int main(void)
{
  	hardware_init();
}
