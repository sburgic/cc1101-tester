#include "cc1101.h"

#include "system.h"

#include <msp430.h>

#define CC_SPI_BEGIN()         do { P3OUT &= ~BIT1; } while (0)
#define CC_SPI_TX(x)           do { UCA1IFG &= ~UCRXIFG; UCA1TXBUF= (x); } \
                                      while (0)
#define CC_SPI_WAIT_DONE()     while (!(UCA1IFG & UCRXIFG))
#define CC_SPI_RX()            UCA1RXBUF
#define CC_SPI_WAIT_MISO_LOW() do { uint8_t count = 200;             \
                                       while( 0 != ( P2IN & BIT5 ))  \
                                       {                             \
                                            __delay_cycles( 5000 );  \
                                            count--;                 \
                                              if ( 0 == count )      \
                                              {                      \
                                                  break;             \
                                              }                      \
                                        }                            \
                                   }while (0)
#define CC_SPI_END()           do { P3OUT |= BIT1; } while (0)


/* 433 MHz, 10 Kbps, 2GFSK, Manchester off*/
static const uint8_t cc_cfg[41] =
{ 0x2E, 0x2E, 0x2E, 0x47, 0xD3, 0x91, CC_BUFF_LEN, 0x0E,  0x05, CC_DEFAULT_ADDR,
  CC_DEFAULT_CH, 0x06, 0x00, 0x10, 0xA7, 0x62, 0xC8, 0x93, 0x16, 0xA2, 0xF8,
  0x34, 0x07, 0x3F, 0x18, 0x16, 0x6C, 0x43, 0x40, 0x91, 0x87, 0x6B, 0xFB, 0x56,
  0x10, 0xE9, 0x2A, 0x00, 0x1F, 0x41, 0x00
};
static const uint8_t cc_patable[8] =
{
    0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0
};

static uint8_t cc_rssi = 0;

static void cc_spi_init( void )
{
    UCA1CTLW0 |= UCSWRST;  /* Software reset */

    UCA1CTLW0 |= UCSYNC;   /* Synchronous mode */
    UCA1CTLW0 |= UCCKPH;   /* Data is captured on the first UCLK edge */
    UCA1CTLW0 &= ~UCCKPL;  /* Clock Polarity 0 */
    UCA1CTLW0 |= UCMSB;    /* MSB first */
    UCA1CTLW0 &= ~UC7BIT;  /* 8-bit data */
    UCA1CTLW0 |= UCMST;    /* Master mode */
    UCA1CTLW0 |= UCMODE_0; /* 3-pin SPI; Manual CS control */
    UCA1CTLW0 |= UCSSEL_2; /* 8 MHz clock (from SMCLK) */

    UCA1BRW = 2; /* Prescaler set to 2 */

    P2SEL0 |= BIT4 + BIT5 + BIT6;
    P3OUT  |= BIT1;
    P3DIR  |= BIT1;
    P2DIR  |= BIT4 + BIT6;
    P2DIR  &= BIT5;

    UCA1CTLW0 &= ~UCSWRST; /* Release from reset */
}

static uint8_t cc_spi_strobe( uint8_t strobe )
{
    uint8_t ret;

    CC_SPI_BEGIN();
    CC_SPI_WAIT_MISO_LOW();
    CC_SPI_TX( strobe );
    CC_SPI_WAIT_DONE();
    ret = CC_SPI_RX();
    CC_SPI_END();

    return ret;
}

static void cc_xfer_burst_single( uint8_t addr, uint8_t* data, uint16_t len )
{
    uint16_t i;

    if ( 0 != ( addr & CC_READ_ACCESS ))
    {
        if ( addr & CC_BURST_ACCESS )
        {
            for ( i = 0; i < len; i++ )
            {
                CC_SPI_TX( 0 );
                CC_SPI_WAIT_DONE();
                *data = CC_SPI_RX();
                data++;
            }
        }
        else
        {
            CC_SPI_TX( 0 );
            CC_SPI_WAIT_DONE();
            *data = CC_SPI_RX();
        }
    }
    else
    {
        if ( 0 != ( addr & CC_BURST_ACCESS ))
        {
            for ( i = 0; i < len; i++ )
            {
                CC_SPI_TX( *data );
                CC_SPI_WAIT_DONE();
                data++;
            }
        }
        else
        {
            CC_SPI_TX( *data );
            CC_SPI_WAIT_DONE();
        }
    }
}

static uint8_t cc_reg_access( uint8_t addr, uint8_t* data, uint16_t len )
{
    uint8_t status; /* Chip status */

    CC_SPI_BEGIN();
    CC_SPI_WAIT_MISO_LOW();

    CC_SPI_TX( addr );
    CC_SPI_WAIT_DONE();

    status = CC_SPI_RX();
    cc_xfer_burst_single( addr, data, len );
    CC_SPI_END();

    return status;
}

static uint8_t cc_reset( void )
{
    uint8_t ret     = 0;
    uint8_t partnum = 0;
    uint8_t ver     = 0;

    cc_spi_strobe( CC_CMD_SRES );
    cc_read_reg( CC_ST_REG_PARTNUM, &partnum, 1 );
    cc_read_reg( CC_ST_REG_VERSION, &ver, 1 );

    if (( 0 != partnum ) || ( 0x14 != ver ))
    {
        ret = 1;
        WDTCTL &= ~WDTHOLD;
    }

    return ret;
}

static void cc_wait_tx_end( void )
{
    uint8_t status;
    int16_t timeout = 200;

    do
    {
        cc_read_reg( CC_ST_REG_MARCSTATE, &status, 1 );
        timeout--;
        wait( 1000 );
    } while (( 0x0D != status ) && ( timeout > 0 ));

    if ( timeout <= 0 )
    {
        cc_spi_strobe( CC_CMD_IDLE );
        cc_spi_strobe( CC_CMD_SFTX );
        cc_spi_strobe( CC_CMD_SRX );
    }
}

static void cc_clear_rx( void )
{
    cc_spi_strobe( CC_CMD_IDLE );
    cc_spi_strobe( CC_CMD_SFRX );
    cc_spi_strobe( CC_CMD_SRX );
}

uint8_t cc_read_reg( uint8_t addr, uint8_t* data, uint8_t len )
{
    return cc_reg_access(( CC_BURST_ACCESS | CC_READ_ACCESS ) | addr
                        , data
                        , len
                        );
}

uint8_t cc_write_reg( uint8_t addr, uint8_t* data, uint8_t len )
{
    return cc_reg_access(( CC_BURST_ACCESS | CC_WRITE_ACCESS ) | addr
                        , data
                        , len
                        );
}

uint8_t cc_init( void )
{
    uint8_t ret;
    uint8_t test_val;

    cc_spi_init();
    ret = cc_reset();

    if ( 0 == ret )
    {
        cc_write_reg( CC_REG_CFG, cc_cfg, 41 );
        cc_write_reg( CC_REG_PATABLE, cc_patable, 8 );

        cc_read_reg( CC_CFG_REG_IOCFG2, &test_val, 1 );

        if ( test_val != cc_cfg[0] )
        {
            ret = 1;
        }
        else
        {
            cc_read_reg( CC_CFG_REG_PKTCTRL1, &test_val, 1 );
            cc_rssi = ( test_val & 0x04 );

            cc_spi_strobe( CC_CMD_SRX );
            wait( 50000 );
        }
    }

    return ret;
}

uint8_t cc_send( uint8_t* data )
{
    uint8_t len;
    uint8_t idx;
    uint8_t status = 1;

    if (( NULL != data ) && ( 0 != data[0] ))
    {
        len = data[0];

        cc_spi_strobe( CC_CMD_IDLE );
        data[0]--;

        if ( len < 64 )
        {
            cc_write_reg( CC_REG_TXFIFO, data, len );
            len = 0;
        }
        else
        {
            cc_write_reg( CC_REG_TXFIFO, data, 64 );
            len -= 64;
            idx  = 64;
        }

        cc_spi_strobe( CC_CMD_STX );

        while ( 0 != len )
        {
            status = cc_spi_strobe( CC_CMD_SNOP );

            if ( 0 != status )
            {
                cc_write_reg( CC_REG_TXFIFO_S, &data[idx++], 1 );
                len--;
            }
            else
            {
                wait( 1000 );
            }
        }

        cc_wait_tx_end();
        status = 0;
    }

    return status;
}

uint8_t cc_receive( uint8_t* data )
{
    uint8_t  bytes = 0;
    uint8_t  l;
    uint8_t  n;
    uint8_t  len;
    uint64_t time_start;
    uint64_t time_end;
    uint8_t  status = 0;
    uint8_t  read;

    if ( NULL != data )
    {
        time_start = get_tick();
        cc_read_reg( CC_ST_REG_RXBYTES, &n, 1 );

        if ( 0 != ( n & 0x80 ))
        {
            cc_clear_rx();
        }
        else
        {
            do
            {
                l = n;
                cc_read_reg( CC_ST_REG_RXBYTES, &n, 1 );
            } while (( n < 2 ) && ( n != l ));

            if ((( n > 0 ) && ( n < 0x80 )) && ( n <= ( CC_BUFF_LEN - 2 )))
            {
                wait( 3000 );
                cc_read_reg( CC_REG_RXFIFO, &len, 1 );
                *data++ = len;
                bytes++;

                if ( len <= ( CC_BUFF_LEN - 3 ))
                {
                    while ( len > 1 )
                    {
                        while ( n < len )
                        {
                            cc_read_reg( CC_ST_REG_RXBYTES, &n, 1 );

                            do
                            {
                                l = n;
                                cc_read_reg( CC_ST_REG_RXBYTES, &n, 1 );
                                time_end = get_tick();

                                if ((( time_end - time_start ) / 1000 ) > 200 )
                                {
                                    cc_clear_rx();
                                    status = 1;
                                    break;
                                }
                                wait( 3000 );
                            } while (( 0 != n ) && ( n != l ));

                            if ( 1 == status )
                            {
                                break;
                            }
                        }

                        if ( 0 == status )
                        {
                            cc_read_reg( CC_REG_RXFIFO, &read, 1 );
                            *data++ = read;
                            len--;
                            bytes++;
                        }
                        else
                        {
                            break;
                        }
                    }

                    if ( 0 == status )
                    {
                        cc_read_reg( CC_REG_RXFIFO, &read, 1 );
                        *data++ = read;
                        bytes++;

                        if ( 0 != cc_rssi )
                        {
                            cc_read_reg( CC_REG_RXFIFO, &read, 1 );
                            *data++ = read;
                            bytes++;

                            cc_read_reg( CC_REG_RXFIFO, &read, 1 );
                            *data= read;
                            bytes++;
                        }
                    }
                }
                else
                {
                    cc_clear_rx();
                }
            }
            else
            {
                if ( 0 != n )
                {
                    cc_clear_rx();
                }
            }
        }
    }

    return bytes;
}

uint8_t cc_crc( uint8_t* data )
{
    uint8_t i;
    uint8_t len;
    uint8_t crc = 0;

    len = *data;

    for ( i = 0; i < len; i++ )
    {
        crc += *data++;
    }

    return crc;
}

int8_t cc_calc_rssi( uint8_t raw )
{
    int8_t rssi = 0;

    if ( raw >= 128 )
    {
        rssi = (( raw - 256 ) / 2 ) - 74;
    }
    else
    {
        rssi = ( raw / 2 ) - 74;
    }

    return rssi;
}
