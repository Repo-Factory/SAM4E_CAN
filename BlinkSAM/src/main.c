/* 
 * @ Zix
 * SAM4E Light Blink
 */

#include "asf.h"

/* 
	There is one yellow LED available on the SAM4E Xplained Pro board that can be turned on and off. The
	LED can be activated by driving the connected I/O line to GND. PD22 Yellow LED
*/
#define LED0_PIO   PIOD
#define LED0_PIN   PIO_PD22

/* 
	The level driven on an I/O line can be determined by writing in the
	Set Output Data Register (PIO_SODR) and the
	Clear Output Data Register (PIO_CODR) 
*/
#define LIGHT_ON()  LED0_PIO->PIO_CODR=LED0_PIN
#define LIGHT_OFF() LED0_PIO->PIO_SODR=LED0_PIN

#define TOGGLE(BIT) BIT^=1 // We like XOR
#define TOGGLE_PERIOD 1000

static uint32_t light_on=0;
static uint32_t tickcount=0;

static void toggle_light(void)
{
	if (light_on)
		LIGHT_OFF();
	else
		LIGHT_ON();
	TOGGLE(light_on);
}

// Interrupt Routine (ISR) called in Response to SysTick Interrupt Request (IQR)
void SysTick_Handler(void)
{
	if (++tickcount==TOGGLE_PERIOD)
	{
		toggle_light();
		tickcount = 0;
	}
}

static void hardware_init(void)
{
	/* 
	 *	When the I/O line is controlled by the PIO Controller, the pin can be configured to be driven. 
	 * This is done by writing the Output Enable Register (PIO_OER) 
	 */
    LED0_PIO->PIO_OER = LED0_PIN;
    LED0_PIO->PIO_CODR = LED0_PIN;
	
	/* 	
	 * Initializes the System Timer and its interrupt, and starts the System Tick Timer.
     *    Counter is in free running mode to generate periodic interrupts.
  	 * \param [in]  ticks  Number of ticks between two interrupts.
  	*/
    SysTick_Config(SystemCoreClock/TOGGLE_PERIOD);
    NVIC_EnableIRQ(SysTick_IRQn);
}

int main(void)
{
  	hardware_init();
}