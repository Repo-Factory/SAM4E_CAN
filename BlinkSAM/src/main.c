/* 
 * SAM4E Light Blink
 */

#include "asf.h"

#define LED0_PIO   PIOD
#define LED0_PIN   PIO_PD22
#define LIGHT_ON()  LED0_PIO->PIO_CODR=LED0_PIN
#define LIGHT_OFF() LED0_PIO->PIO_SODR=LED0_PIN

#define TOGGLE(BIT) BIT^=1
#define TOGGLE_PERIOD 1000UL

#define DISABLE_WATCHDOG() WDT->WDT_MR = WDT_MR_WDDIS

static uint32_t light_on=0;
static uint32_t tickcount=0;

/**
 * Initialize LED and interrupt handler
 */

static void toggle_light(void)
{
	if (light_on)
		LIGHT_OFF();
	else
		LIGHT_ON();
	TOGGLE(light_on);
}

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
    // Enable Interrupt Handling
    SysTick_Config(SystemCoreClock/TOGGLE_PERIOD);
    NVIC_EnableIRQ(SysTick_IRQn);

    LED0_PIO->PIO_OER = LED0_PIN;
    LED0_PIO->PIO_CODR = LED0_PIN;
}

int main(void)
{
  	DISABLE_WATCHDOG();
  	hardware_init();
}