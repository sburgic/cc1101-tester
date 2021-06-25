#include <msp430.h> 

#include "cc1101.h"
#include "lcd2wire.h"
#include "rf_talk.h"
#include "system.h"
#include "uart.h"

#include <stdio.h>

#ifdef CC_SENDER
    #define LED_INIT()    do { P3OUT &= ~BIT0; P3DIR |= BIT0; \
                              P3OUT &= ~BIT2; P3DIR |= BIT2; } while (0)
    #define LED_RED_ON()  do { P3OUT |= BIT0; } while (0)
    #define LED_RED_OFF() do { P3OUT &= ~BIT0; } while (0)
    #define LED_GN_ON()   do { P3OUT |= BIT2; } while (0)
    #define LED_GN_OFF()  do { P3OUT &= ~BIT2; } while (0)

    #define PB_INIT()  do { P2DIR &= ~BIT2; } while (0)
    #define PB_READ(x) do { x = (uint8_t)( 0 != ( P2IN & BIT2 )) > 0 ? 1 : 0; }\
                            while (0)
#endif

int main( void )
{
    uint8_t status;
    uint8_t marcstate = 0;
    uint8_t payload[CC_BUFF_LEN] = {0};

    system_init();

#ifdef CC_TESTER
    uint8_t   ds_out[64] = {0};
    uint8_t   rx_cnt     = 0;
    int8_t    rssi       = 0;

    lcd_init();
    uart_init();
#else
    uint8_t crc;
    uint8_t pb_state;
    int16_t timeout;
    uint8_t ack_flag;
    int8_t  repeat;
    uint8_t ack[RF_TALK_ACK_SIZE] = {0};
    uint8_t i;

    payload[0] = 6;  /* Length total */
    payload[1] = 2;  /* Destination address */
    payload[2] = 1;  /* Source address */
    payload[3] = 50; /* CMD header */
    payload[4] = 0;  /* CMD data */
    payload[5] = 0;  /* CRC */

    LED_INIT();
    PB_INIT();
#endif

    do
    {
        status = cc_init();
        cc_read_reg( CC_ST_REG_MARCSTATE, &marcstate, 1 );
    } while ( 0x0D != marcstate );

    if ( 0 != status )
    {
#ifdef CC_TESTER
        lcd_puts_xy_cl( "CC init failed.", 0, 0 );
#else
        LED_RED_ON();
#endif
        wait( 100000 );
        WDTCTL &= ~WDTHOLD;
    }
    else
    {
#ifdef CC_TESTER
        lcd_puts_xy_cl( "CC1101 TESTER", 1, 0 );
        lcd_puts_xy_cl( "SW ver.1.0", 3, 1 );
        wait( 2000000 );
        lcd_clear();
        lcd_puts_xy_cl( "433MHz 10Kbps", 0, 0 );
        lcd_puts_xy_cl( "2GFSK M.off", 0, 1 );
#else
        LED_GN_ON();
        wait( 500000 );
        LED_GN_OFF();
#endif
    }

    for(;;)
    {
#ifdef CC_SENDER

        PB_READ( pb_state );

        if ( 0 == pb_state )
        {
            wait( 75000 );
            {
                PB_READ( pb_state );
                if ( 0 == pb_state )
                {
                    do
                    {
                        PB_READ( pb_state );
                    } while ( 0 == pb_state );

                    if ( 0 == payload[4] )
                    {
                        payload[4] = 1;
                    }
                    else
                    {
                        payload[4] = 0;
                    }

                    payload[0] = 6;
                    crc = cc_crc( payload );
                    payload[5] = 256 - crc;

                    repeat = 2;
                    while ( repeat > 0 )
                    {
                        LED_GN_ON();
                        status = cc_send( payload );

                        if ( 0 == status )
                        {
                            ack_flag = 0;
                            timeout  = 200;

                            for ( i = 0; i < RF_TALK_ACK_SIZE; i++ )
                            {
                                ack[i] = 0;
                            }

                            LED_GN_OFF();

                            do
                            {
                                status = cc_receive( ack );

                                if ( 0 != status )
                                {
                                    if (( RF_TALK_ACK_SIZE - 1 ) == ack[0])
                                    {
                                        ack_flag = 1;
                                        LED_RED_OFF();
                                        LED_GN_ON();
                                        wait( 50000 );
                                        break;
                                    }
                                    else
                                    {
                                        LED_RED_ON();
                                        LED_GN_OFF();
                                    }
                                }

                                wait( 1000 );
                                timeout--;
                                repeat--;
                            } while (( timeout > 0 ) && ( 0 == ack_flag ));

                            LED_RED_OFF();
                        }
                        else
                        {
                            LED_RED_ON();
                            wait( 75000 );
                            LED_RED_OFF();
                        }

                        if ( 1 == ack_flag )
                        {
                            wait( 500000 );
                            LED_GN_OFF();
                            break;
                        }
                        else
                        {
                            LED_RED_ON();
                            wait( 200000 );
                            LED_RED_OFF();
                        }
                    }
                }
            }
        }
#else
        status = cc_receive( payload );

        if ( 0 != status )
        {
            uint8_t len = payload[0] + 2;

            rx_cnt++;
            sprintf((char*) ds_out, "Rx:%d", rx_cnt );
            lcd_puts_xy_cl( ds_out, 0, 0 );
            sprintf((char*) ds_out, "Len:%d", payload[0] );
            lcd_puts_xy_cl( ds_out, 8, 0 );
            rssi = cc_calc_rssi( payload[len - 1] );
            sprintf((char*) ds_out, "RSSI:%ddBm", rssi );
            lcd_puts_xy_cl( ds_out, 0, 1 );
            sprintf((char*) ds_out,
                     "Packet received. Len = %d, Destination = %d, "
                     "Source = %d, CRC = %d.\r\n"
                     , payload[0]
                     , payload[1]
                     , payload[2]
                     , payload[len - 3]
                     );
            uart_puts( ds_out );

            rft_send_ack( payload[2] );
            sprintf((char*) ds_out
                   , "ACK sent. Destination = %d.\r\n"
                   , payload[2]
                   );
            uart_puts( ds_out );
        }
#endif
    }

    return 0;
}
