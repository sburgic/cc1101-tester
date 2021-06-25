#include "lcd2wire.h"

#include "system.h"

#if (LCD2WIRE_SCREEN_SIZE == 1)
    #define LCD_TOTAL_ROWS    2
    #define LCD_TOTAL_COLUMNS 16
#elif (LCD2WIRE_SCREEN_SIZE == 2)
    #define LCD_TOTAL_ROWS    4
    #define LCD_TOTAL_COLUMNS 20
#else
    #error Invalid LCD2WIRE_SCREEN_SIZE !!
#endif

/* Standard commands should work with most common devices */
#define LCD_CMD_CLEAR_LCD        0x01 /* Clear Display */
#define LCD_CMD_RETURN_HOME      0x02 /* Cursor to Home position */
#define LCD_CMD_ENTRY_MODE       0x06 /* Normal entry mode */
#define LCD_CMD_ENTRY_MODE_SHIFT 0x07 /* With shift */
#define LCD_CMD_SYSTEM_SET_8_BIT 0x38 /* 8 bit data mode 2 line */
#define LCD_CMD_SYSTEM_SET_4_BIT 0x28 /* 4 bit data mode 2 line */
#define LCD_CMD_DISPLAY_ON       0x0C /* Switch ON Display */
#define LCD_CMD_DISPLAY_OFF      0x08 /* Cursor plus blink */
#define LCD_CMD_SET_DD_LINE1     0x80 /* Line 1 */
#define LCD_CMD_SET_DD_LINE2     0xC0 /* Line 2 */
#define LCD_CMD_SET_DD_LINE3     0x94 /* Line 3 */
#define LCD_CMD_SET_DD_LINE4     0xD4 /* Line 4 */
#define LCD_CMD_WRITE_DATA       0x00 /* With RS = 1 */
#define LCD_CMD_CURSOR_ON        0x0E /* Switch Cursor ON */
#define LCD_CMD_CURSOR_ON_BLINK  0x0F /* Switch Cursor BLINKING ON */
#define LCD_CMD_CURSOR_OFF       0x0C /* Switch Cursor OFF */

/* LCD Procedures for HD44780 */
#define LCD_CURSOR(X)       lcd_write_cmd(((x)&0x7F)|0x80)
#define LCD_CLEAR_ALL()     lcd_write_cmd(LCD_CMD_CLEAR_LCD)
#define LCD_GOTO(X)         lcd_write_cmd(LCD_CMD_SET_DD_LINE1+(x))
#define LCD_GOTOXY(X,Y)     lcd_write_cmd(LCD_CMD_SET_DD_LINE1+x+(0x40*y))
#define LCD_CURSOR_RIGHT()  lcd_write_cmd(0x14)
#define LCD_CURSOR_LEFT()   lcd_write_cmd(0x10)
#define LCD_DISPLAY_SHIFT() lcd_write_cmd(0x1C)
#define LCD_HOME()          lcd_write_cmd(LCD_CMD_RETURN_HOME)

#define LCD_DELAY_WAIT (5000)

static void lcd_nibble_out( uint8_t nibble, uint8_t rs )
{
    int i = 0;

    LCD2WIRE_DATA_PIN_RESET();

    __delay_cycles(10);

    for ( i = 6; i > 0; --i )
    {
        LCD2WIRE_CLK_PIN_SET();
        __delay_cycles(10);
        LCD2WIRE_CLK_PIN_RESET();
        __delay_cycles(10);
    }

    LCD2WIRE_DATA_PIN_SET();
    __delay_cycles(10);
    LCD2WIRE_CLK_PIN_SET();
    __delay_cycles(10);
    LCD2WIRE_CLK_PIN_RESET();
    __delay_cycles(10);

    if ( 0 == rs )
    {
        LCD2WIRE_DATA_PIN_RESET();
    }
    else
    {
        LCD2WIRE_DATA_PIN_SET();
    }

    __delay_cycles(10);
    LCD2WIRE_CLK_PIN_SET();
    __delay_cycles(10);
    LCD2WIRE_CLK_PIN_RESET();
    __delay_cycles(10);

    for ( i = 0x08; i > 0; i >>= 1 )
    {

        if ( 0 != ( i & nibble ))
        {
            LCD2WIRE_DATA_PIN_SET();
        }
        else
        {
            LCD2WIRE_DATA_PIN_RESET();
        }

        __delay_cycles(10);
        LCD2WIRE_CLK_PIN_SET();
        __delay_cycles(10);
        LCD2WIRE_CLK_PIN_RESET();
        __delay_cycles(10);
    }

    LCD2WIRE_DATA_PIN_SET();
    __delay_cycles(10);
    LCD2WIRE_DATA_PIN_RESET();
    __delay_cycles(10);
}

static void lcd_write_cmd ( uint8_t byte )
{
    lcd_nibble_out( byte >> 4, 0 );
    __delay_cycles(10);
    lcd_nibble_out( byte & 0x0F, 0 );
    wait( LCD_DELAY_WAIT );
}

static void lcd_goto_rc ( uint8_t row, uint8_t col )
{
    if ( 0 == row )
    {
        lcd_write_cmd( LCD_CMD_SET_DD_LINE1 + col );
    }
    else if ( 1 == row )
    {
        lcd_write_cmd( LCD_CMD_SET_DD_LINE2 + col );
    }
#if ( LCD2WIRE_SCREEN_SIZE == 2 )
    else if ( 2 == row )
    {
        lcd_write_cmd( LCD_CMD_SET_DD_LINE3 + col );
    }
    else if ( 3 == row )
    {
        lcd_write_cmd( LCD_CMD_SET_DD_LINE4 + col );
    }
#endif
}

static void lcd_putch( uint8_t byte )
{
    lcd_nibble_out( byte >> 4, 1 );
    __delay_cycles(10);
    lcd_nibble_out( byte & 0x0F, 1 );
    __delay_cycles(10);
    __delay_cycles(10);
}

void lcd_putch_xy( uint8_t byte, uint8_t x, uint8_t y )
{
    lcd_goto_rc( y, x );
    lcd_putch( byte );
}

uint8_t lcd_puts_xy( const uint8_t* str, uint8_t x, uint8_t y )
{
    uint8_t i = 0;

    if ( NULL != str )
    {
        lcd_goto_rc( y, x );

        for ( i = 0; ( str[i]!='\0' ) && ( i < LCD_TOTAL_COLUMNS ) ; i++ )
        {
            lcd_putch( str[i] );
        }
    }

    return i;
}

void lcd_puts_xy_cl( const uint8_t* str, uint8_t x, uint8_t y )
{
    int8_t i;

    i = lcd_puts_xy( str, x, y );

    while ( i++ < LCD_TOTAL_COLUMNS )
    {
        lcd_putch(' ');
    }
}

void lcd_init( void )
{
    LCD2WIRE_GPIO_INIT();

    /* 8 bit interface/and wait 5ms */
    lcd_write_cmd( 0xF3 );
    wait( LCD_DELAY_WAIT );
    /* 8 bit interface/and wait 160uS */
    lcd_write_cmd( 0xF3 );
    wait( 160 );
    /* {8 bit interface/and wait 160uS */
    lcd_write_cmd( 0xF3 );
    wait( 160 );
    /* {4 bit interface} */
    lcd_write_cmd( 0xF2 );
    wait( LCD_DELAY_WAIT );
    lcd_write_cmd( 0x20+0x08 ); /* 4 bit, 2 rows, 5x7 */
    wait( LCD_DELAY_WAIT );
    lcd_write_cmd( 0x06 ); /* Increment position, display shift off */
    lcd_write_cmd( 0x0C ); /* Display = On, Cursor = Off, Blink = Off */

    lcd_clear();
    wait( LCD_DELAY_WAIT );
    LCD_HOME();
}

void lcd_clear( void )
{
    LCD_CLEAR_ALL();
}

void lcd_clear_row( uint8_t row )
{
    uint8_t i = 0;

    if ( row < LCD_TOTAL_ROWS )
    {
        while( i < LCD_TOTAL_COLUMNS )
        {
            lcd_putch_xy( ' ', i, row );
            i++;
        }
    }
}
