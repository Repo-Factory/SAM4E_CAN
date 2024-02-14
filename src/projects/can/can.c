#include "asf.h"
#include "can_asf.h"
#include "led_f.h"

#define CAN1_PIO        	   PIOC
#define CAN1_RX_PIN     	   PIO_PC12
#define CAN1_TX_PIN     	   PIO_PC15
#define CAN1_IRQ	    	   CAN1_IRQn
#define CAN1_BASE       	   CAN1
#define CAN1_PID        	   ID_CAN1

volatile Can* can = (volatile Can*)CAN1_BASE;

void setup_can(void);
    void set_prph_clk(void);
    void set_isr_line(void);
    void set_can_enable(void);
void setup_mailbox(void);

void setup_can(void)
{
    set_prph_clk();
    set_isr_line();
    set_can_enable();
}

void setup_mailbox(void)
{

}

int main(void)
{
    for (;;)
    {
        
    }
}