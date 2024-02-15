#include "asf.h"
#include "quick_blink.h"

#define TOGGLE(BIT) BIT^=1 // We like XOR
#define TOGGLE_PERIOD 1000

static uint32_t light_on=0;
static uint32_t tickcount=0;
static uint32_t stopcount=0;

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
	if (++tickcount==100)
	{
		toggle_light();
		tickcount = 0;
	}
    if (++stopcount==1000)
    {
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
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

void quick_blink(void)
{
    hardware_init();
}