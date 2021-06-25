#include "ds18b20.h"

/* How many bytes to read with scratchpad command */
#define DS_DATA_LEN (9)

static Ds_Hdl_t ds_hdl;

static int16_t ds18b20_read_temp( uint8_t dev_idx )
{
    int16_t temperature = DS_SENSOR_ERROR;
    uint8_t i;
    uint8_t crc = 0;
    uint8_t data[DS_DATA_LEN] = {0};

    ow_match_rom( &ds_hdl.ow_hdl, dev_idx );
    ow_write_byte( &ds_hdl.ow_hdl, OW_CMD_READ_SCHRATCHPAD );

    for ( i = 0; i < DS_DATA_LEN; i++ )
    {
        data[i] = ow_read_byte( &ds_hdl.ow_hdl );
    }

    crc = ow_crc8( data, 8 );

    if ( crc == data[8] )
    {
        temperature = data[0] | ( data[1] << 8 );
    }

    return temperature;
}

void ds18b20_init( void )
{
    ow_init( &ds_hdl.ow_hdl );
}

void ds18b20_update( void )
{
    bool_t  bret;
    uint8_t i;

    ow_reset_search_state( &ds_hdl.ow_hdl );

    bret             = ow_reset( &ds_hdl.ow_hdl );
    ds_hdl.no_of_dev = ow_list_all_devices( &ds_hdl.ow_hdl );

    if (( FALSE != bret ) && ( 0 != ds_hdl.no_of_dev ))
    {
        bret = ow_convert_temp( &ds_hdl.ow_hdl );
    }

    if ( FALSE != bret )
    {
        for ( i = 0; i < ds_hdl.no_of_dev; i++ )
        {
            ds_hdl.last_temperature[i] = ds18b20_read_temp( i );
        }
    }

    for ( i = ds_hdl.no_of_dev; i < OW_MAX_DEVICES; i++ )
    {
        ds_hdl.last_temperature[i] = DS_SENSOR_ERROR;
    }
}

void ds18b20_temp_to_string( uint8_t* buff, int16_t temperature )
{
    int16_t fraction;
    uint8_t x;

    if ( NULL != buff )
    {
        if ( DS_SENSOR_ERROR != temperature )
        {
            /* Check if negative value */
            if ( 0 != ( temperature & 0x8000 ))
            {
                *buff++     = '-';
                temperature = ~temperature + 1;
            }

            fraction  = temperature & 0x000F; /* Last 4 bits are decimals */

            /* Use one decimal place */
            fraction *= 625;
            fraction /= 1000;

            x = ( temperature >> 4 ) / 100;
            if ( 0 != x )
            {
                *buff++ = x + '0';
            }

            x = ( temperature >> 4 ) / 10;
            if ( 0 != x )
            {
                *buff++ = ( x % 10 ) + '0';
            }

            *buff++ = (( temperature >> 4 ) % 10 ) + '0';
            *buff++ = '.';
            *buff++ = fraction + '0';
            *buff   = '\0';
        }
        else
        {
            *buff++ = 'E';
            *buff++ = 'r';
            *buff++ = 'r';
            *buff++ = '.';
            *buff   = '\0';
        }
    }
}

Ds_Hdl_t* ds18b20_get_hdl( void )
{
    return &ds_hdl;
}
