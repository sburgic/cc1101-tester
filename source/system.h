#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <stdint.h>

#ifndef FALSE
    #define FALSE (0)
    #define TRUE !(FALSE)
#endif

#ifndef NULL
    #define NULL ((void*)0)
#endif

typedef uint8_t bool_t;

void system_init( void );
void timer2_A2_irq_hdl( void );
uint64_t get_tick( void );
void wait( uint64_t us );

#endif /* __SYSTEM_H__ */
