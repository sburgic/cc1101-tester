#ifndef __ONE_WIRE_H__
#define __ONE_WIRE_H__

#include "system.h"

#include <msp430.h>
#include <stdint.h>

#define OW_ROM_SIZE     (8)  /* 1-Wire device ROM size in bytes */
#define OW_MAX_DEVICES  (10) /* Maximum number of devices
                              * on 1-Wire bus
                              */

/* 1-Wire basic commands */
#define OW_CMD_SEARCH_ROM       (0xF0) /* Multiple devices on the bus */
#define OW_CMD_READ_ROM         (0x33) /* Single device on the bus    */
#define OW_CMD_SKIP_ROM         (0xCC)
#define OW_CMD_MATCH_ROM        (0x55)
#define OW_CMD_CONVERT_TEMP     (0x44)
#define OW_CMD_READ_SCHRATCHPAD (0xBE)

/* Check "Maxim Integrated 1-Wire Application Note 126" for details.
 * Delay values need to be updated for other CPU frequencies.
 */
#define OW_DELAY_A (2)   /* 6us   (+1.0)   */
#define OW_DELAY_B (47)  /* 64us  (+7.5)   */
#define OW_DELAY_C (44)  /* 60us  (+7.5)   */
#define OW_DELAY_D (5)   /* 10us  (+2.5)   */
#define OW_DELAY_E (4)   /* 9us   (+1.0)   */
#define OW_DELAY_F (40)  /* 55us  (+7.0)   */
#define OW_DELAY_H (346) /* 480us (+70.0)  */
#define OW_DELAY_I (51)  /* 70us  (+8.5)   */
#define OW_DELAY_J (310) /* 410us (+40.0)  */

#define OW_SET_INPUT() do { P2DIR &= ~BIT2; } while (0)
#define OW_SET_OUTPUT() do { P2DIR |= BIT2; } while (0)
#define OW_WRITE_LOW() do { P2OUT &= ~BIT2; } while (0)
#define OW_WRITE_HIGH() do { P2OUT |= BIT2; } while (0)
#define OW_READ_PIN(x) do { x = (uint8_t)( 0 != ( P2IN & BIT2 )) > 0 ? 1 : 0; }\
                       while (0)

/* Main 1-Wire structure */
typedef struct
{
    uint8_t rom[8];
    uint8_t last_discrepancy;
    uint8_t last_family_discrepancy;
    bool_t  last_device_flag;
} one_wire_t;


/* Initialize 1-Wire bus */
void ow_init( one_wire_t* ow );
/* Send reset pulse to 1-Wire bus */
bool_t ow_reset( one_wire_t* ow );
/* Reset search state */
void ow_reset_search_state( one_wire_t* ow );
/* Search for devices on 1-Wire bus */
bool_t ow_search( one_wire_t* ow, uint8_t* addr_buff );
/* Write byte to 1-Wire bus */
void ow_write_byte( one_wire_t* ow, uint8_t byte );
/* Read byte from 1-Wire bus */
uint8_t ow_read_byte( one_wire_t* ow );
/* Calculate CRC8 */
uint8_t ow_crc8( uint8_t* buff, uint8_t length );
/* List all devices on 1-Wire bus */
uint8_t ow_list_all_devices( one_wire_t* ow );
/* Match specific device using ROM */
bool_t ow_match_rom( one_wire_t* ow
                   , uint8_t     dev_idx
                   );
/* Issue convert temperature instruction to all devices on 1-Wire bus */
bool_t ow_convert_temp( one_wire_t* ow );

#endif /* __ONE_WIRE_H__ */
