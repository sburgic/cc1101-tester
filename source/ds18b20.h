#ifndef __DS18B20_H__
#define __DS18B20_H__

#include "one_wire.h"

typedef struct
{
    one_wire_t ow_hdl;
    uint8_t    no_of_dev;
    int16_t    last_temperature[OW_MAX_DEVICES];
} Ds_Hdl_t;

/* Sensor error - value that never appears in measurement */
#define DS_SENSOR_ERROR ((int16_t)0xFF80)

/* Initialize DS18B20 */
void ds18b20_init( void );

/* Read temperature of all attached devices */
void ds18b20_update( void );

/* Convert measured temperature into string */
void ds18b20_temp_to_string( uint8_t* buff, int16_t temperature );

/* Get DS18B20 handle */
Ds_Hdl_t* ds18b20_get_hdl( void );

#endif /* __DS18B20_H__ */
