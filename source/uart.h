#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

#define UART_BUFF_SIZE 128

void uart_init( void );
void uart_send( uint8_t* data, uint16_t len );
void uart_puts( uint8_t* data );
void uart_receive_byte( uint8_t byte );

#endif /* __UART_H__ */
