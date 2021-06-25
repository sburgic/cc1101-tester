#include "one_wire.h"

/* Wait for pin to become stable retries count */
#define OW_RESET_RETRIES_CNT             ((uint8_t)125)

/* Wait cycles used for delay in us */
static inline void ow_wait_cycles( uint32_t n )
{
    volatile int32_t i;

    i = n;

    while ( i > 0 )
    {
        __delay_cycles( 1 );
        i--;
    }
}

/* 1-Wire CRC8 lookup table */
static const uint8_t crc_lookup_tbl[] =
{
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
};

static uint8_t found_rom[OW_MAX_DEVICES][OW_ROM_SIZE];

static void ow_enable_it( void )
{
    _enable_interrupts();
}

static void ow_disable_it( void )
{
    _disable_interrupts();
}

static void ow_write_bit( one_wire_t* ow, uint8_t bit )
{
    if ( FALSE != ( bit & 1 ) )
    {
        ow_disable_it();
        OW_SET_OUTPUT();
        OW_WRITE_LOW();
        ow_wait_cycles( OW_DELAY_A );
        OW_WRITE_HIGH();
        ow_enable_it();
        ow_wait_cycles( OW_DELAY_B );
    }
    else
    {
        ow_disable_it();
        OW_SET_OUTPUT();
        OW_WRITE_LOW();
        ow_wait_cycles( OW_DELAY_C );
        OW_WRITE_HIGH();
        ow_enable_it();
        ow_wait_cycles( OW_DELAY_D );
    }
}

static uint8_t ow_read_bit( one_wire_t* ow )
{
    uint8_t read;

    ow_disable_it();
    OW_SET_OUTPUT();
    OW_WRITE_LOW();
    ow_wait_cycles( OW_DELAY_A );
    OW_WRITE_HIGH(); /* Release the bus */
    ow_wait_cycles( OW_DELAY_E );
    OW_SET_INPUT();

    OW_READ_PIN( read );
    ow_enable_it();
    ow_wait_cycles( OW_DELAY_F );

    return read;
}

void ow_init( one_wire_t* ow )
{
    if ( NULL != ow )
    {
        ow_reset_search_state( ow );
    }
}

bool_t ow_reset( one_wire_t* ow )
{
    bool_t  ret = TRUE;
    uint8_t retries = OW_RESET_RETRIES_CNT;
    uint8_t read;

    ow_disable_it();
    OW_SET_INPUT();
    ow_enable_it();

    /* Wait for pin to become stable */
    do
    {
        --retries;
        ow_wait_cycles( OW_DELAY_A );
        OW_READ_PIN( read );

        if ( 0 == retries )
        {
            ret = FALSE;
            break;
        }
    }
    while ( 0 == read );

    if ( FALSE != ret )
    {
        ow_disable_it();
        OW_SET_OUTPUT();
        OW_WRITE_LOW();
        ow_enable_it();
        ow_wait_cycles( OW_DELAY_H );
        ow_disable_it();

        /* Wait for presence */
        OW_SET_INPUT();
        ow_wait_cycles( OW_DELAY_I );

        /* Read presence */
        OW_READ_PIN( read );
        ret  = !read;

        ow_enable_it();
        ow_wait_cycles( OW_DELAY_J );
    }

    return ret;
}

void ow_write_byte( one_wire_t* ow, uint8_t byte )
{
    uint8_t i;

    for ( i = 0; i < 8; i++ )
    {
        ow_write_bit( ow, ( byte & 0x01 ));
        byte >>= 1;
    }
}

uint8_t ow_read_byte( one_wire_t* ow )
{
    uint8_t i;
    uint8_t ret = 0;
    uint8_t bit;

    for ( i = 0x01; 0 != i; i <<= 1 )
    {
        bit = ow_read_bit( ow );

        if ( 0 != bit )
        {
            ret |= i;
        }
    }

    return ret;
}

void ow_reset_search_state( one_wire_t* ow )
{
    uint8_t i;

    ow->last_discrepancy        = 0;
    ow->last_device_flag        = FALSE;
    ow->last_family_discrepancy = 0;

    for ( i = 0; i < 8; i++ )
    {
        ow->rom[i] = 0;
    }
}

bool_t ow_search( one_wire_t* ow, uint8_t* addr_buff )
{
    bool_t  ret = FALSE;
    uint8_t id_bit_number   = 1;
    uint8_t last_zero       = 0;
    uint8_t rom_byte_number = 0;
    uint8_t rom_byte_mask   = 1;
    uint8_t i;
    uint8_t id_bit;
    uint8_t cmp_id_bit;
    uint8_t search_direction;

    if ( FALSE == ow->last_device_flag )
    {
        ret = ow_reset( ow );
        if ( FALSE == ret )
        {
            ow_reset_search_state( ow );
        }
    }

    if ( FALSE != ret )
    {
        ow_write_byte( ow, OW_CMD_SEARCH_ROM );

        do
        {
            id_bit = ow_read_bit( ow );
            cmp_id_bit = ow_read_bit( ow );

            if (( 1 == id_bit ) && ( 1 == cmp_id_bit ))
            {
                break;
            }
            else
            {
                if ( id_bit != cmp_id_bit )
                {
                    search_direction = id_bit;
                }
                else
                {
                    if ( id_bit_number < ow->last_discrepancy )
                    {
                        search_direction = ((ow->rom[rom_byte_number]
                                         & rom_byte_mask) > 0);
                    }
                    else
                    {
                        search_direction =
                                    ( id_bit_number == ow->last_discrepancy );
                    }

                    if ( 0 == search_direction )
                    {
                        last_zero = id_bit_number;

                        if ( last_zero < 9 )
                        {
                            ow->last_family_discrepancy = last_zero;
                        }
                    }
                }

                if ( 1 == search_direction )
                {
                    ow->rom[rom_byte_number] |= rom_byte_mask;
                }
                else
                {
                    ow->rom[rom_byte_number] &= ~rom_byte_mask;
                }

                ow_write_bit( ow, search_direction );
                id_bit_number++;
                rom_byte_mask <<= 1;

                if ( 0 == rom_byte_mask )
                {
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        } while ( rom_byte_number < 8 );

        if ( id_bit_number >= 65 )
        {
            ow->last_discrepancy = last_zero;
            if ( 0 == ow->last_discrepancy )
            {
                ow->last_device_flag = TRUE;
            }

            ret = TRUE;
        }

        if (( FALSE == ret ) || ( 0 == ow->rom[0] ))
        {
            ow->last_discrepancy        = 0;
            ow->last_device_flag        = FALSE;
            ow->last_family_discrepancy = 0;
            ret = FALSE;
        }

        for ( i = 0; i < 8; i++ )
        {
            addr_buff[i] = ow->rom[i];
        }
    }

   return ret;
}

uint8_t ow_list_all_devices( one_wire_t* ow )
{
    bool_t  b_ret;
    uint8_t i = 0;
    uint8_t no_of_dev = 0;
    uint8_t ow_dev_rom[OW_ROM_SIZE] = {0};

    while ( i < OW_MAX_DEVICES )
    {
        b_ret = ow_search( ow, ow_dev_rom );

        if ( FALSE == b_ret )
        {
            break;
        }

        for ( i = 0; i < OW_ROM_SIZE; i++ )
        {
            found_rom[no_of_dev][i] = ow_dev_rom[i];
        }

        no_of_dev++;
    }

    return no_of_dev;
}

bool_t ow_match_rom( one_wire_t* ow
                   , uint8_t     dev_idx
                   )
{
    bool_t  ret = FALSE;
    uint8_t i;

    ret = ow_reset( ow );

    if ( FALSE != ret )
    {
        ow_write_byte( ow, OW_CMD_MATCH_ROM );

        for ( i = 0; i < OW_ROM_SIZE; i++ )
        {
            ow_write_byte( ow, found_rom[dev_idx][i] );
        }
    }

    return ret;
}

bool_t ow_convert_temp( one_wire_t* ow )
{
    bool_t  ret = FALSE;

    ret = ow_reset( ow );

    if ( FALSE != ret )
    {
        ow_write_byte( ow, OW_CMD_SKIP_ROM );
        ow_write_byte( ow, OW_CMD_CONVERT_TEMP );
    }

    return ret;
}

uint8_t ow_crc8( uint8_t* buff, uint8_t length )
{
    uint8_t crc = 0;

    while ( length-- )
    {
        crc = crc_lookup_tbl[crc ^ *buff++];
    }

    return crc;
}
