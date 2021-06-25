#ifndef __LCD2WIRE_H__
#define __LCD2WIRE_H__

#include <msp430.h>
#include <stdint.h>

#define LCD2WIRE_SCREEN_SIZE 1

#ifndef LCD2WIRE_SCREEN_SIZE
    #error LCD2WIRE_SCREEN_SIZE not defined !!
#else
    #if ((LCD2WIRE_SCREEN_SIZE < 0 ) || ( LCD2WIRE_SCREEN_SIZE > 2 ))
        #error Unsuported LCD screen size !!
    #endif
#endif

#define LCD2WIRE_GPIO_INIT() do { P3OUT &= ~BIT0; P3DIR |= BIT0; \
                                  P3OUT &= ~BIT2; P3DIR |= BIT2; } while (0)

#define LCD2WIRE_DATA_PIN_SET() do { P3OUT |= BIT0; } while (0)
#define LCD2WIRE_DATA_PIN_RESET() do { P3OUT &= ~BIT0; } while (0)
#define LCD2WIRE_CLK_PIN_SET() do { P3OUT |= BIT2; } while (0)
#define LCD2WIRE_CLK_PIN_RESET() do { P3OUT &= ~BIT2; } while (0)

/* Print a character on display at defined position */
void lcd_putch_xy( uint8_t byte, uint8_t x, uint8_t y );
/* Print string on display at defined position */
uint8_t lcd_puts_xy( const uint8_t* str, uint8_t x, uint8_t y );
/* Print string on display at defined position and clear the rest of row */
void lcd_puts_xy_cl( const uint8_t* str, uint8_t x, uint8_t y );
/* Initialize LCD display */
void lcd_init( void );
/* Clear display */
void lcd_clear( void );
/* Clear row */
void lcd_clear_row( uint8_t row );

#endif /* __LCD2WIRE_H__ */
