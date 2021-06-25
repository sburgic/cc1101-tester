#include "uart.h"

#include <msp430.h>

#define UART_GPIO_INIT() do { P1SEL0 |= BIT4; P1SEL1 &= ~BIT4; \
                              P1SEL0 |= BIT5; P1SEL1 &= ~BIT5; } while (0)

static uint8_t  uart_buff[UART_BUFF_SIZE] = {0};
static uint16_t uart_idx = 0;

static void uart_send_byte( uint8_t byte )
{
     while(!( UCA0IFG & UCTXIFG ));
     UCA0TXBUF = byte;
}

void uart_init( void )
{
    UART_GPIO_INIT();

    UCA0CTLW0 |= UCSWRST;  /* Software reset */
    UCA0CTLW0 |= UCSSEL_2; /* 8 MHz clock (from SMCLK) */

    /* 115200 bps
     * Note: See User Guide, chapter 22.3.13 Typical Baud Rates and Errors.
     */
    UCA0BR0   = 4;
    UCA0BR1   = 0;
    UCA0MCTLW = 0x5500 | UCOS16 | UCBRF_5;

    UCA0CTLW0 &= ~UCSWRST; /* Release from reset */
    UCA0IE    |= UCRXIE;   /* Rx interrupts enable */
}

void uart_send( uint8_t* data, uint16_t len )
{
    uint16_t i;

    for ( i = 0; i < len; i++ )
    {
        uart_send_byte( data[i] );
    }
}

void uart_puts( uint8_t* data )
{
    while( *data )
    {
        uart_send_byte( *data++ );
    }
}

void uart_receive_byte( uint8_t byte )
{
    if ( uart_idx < UART_BUFF_SIZE )
    {
        uart_buff[uart_idx++] = byte;
    }
}
