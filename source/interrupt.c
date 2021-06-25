#include <msp430.h>

#include "system.h"
#include "uart.h"

uint8_t rx_rec;

#pragma vector=TIMER2_A0_VECTOR
__interrupt void Timer2_A2( void )
{
    timer2_A2_irq_hdl();
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR( void )
{
    uart_receive_byte( UCA0RXBUF );
}
