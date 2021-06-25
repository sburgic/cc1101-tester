#include "system.h"

#include <msp430.h>

static volatile uint32_t us_time_high = 0;
static volatile uint16_t us_time_low  = 0;

static void clock_init( void )
{
    FRCTL0 = FRCTLPW | NWAITS1; /* Configure FRAM wait state
                                 * if MCLK is greater then 8 MHz
                                 */

    __bis_SR_register( SCG0 ); /* Disable FLL */

    CSCTL3  = SELREF__XT1CLK;  /* Set XT1CLK as FLL reference source */
//    CSCTL3  = SELREF__REFOCLK;
    CSCTL0  = 0x00;            /* Clear DCO and MOD bits */
    CSCTL1 |= DCORSEL_5;       /* Set DCO = 16MHz */
    CSCTL2  = FLLD_0 + 487;    /* DCOCLKDIV = 16MHz */
    CSCTL5 |= DIVS__2;         /* SMCLK = 8 MHz */

    __delay_cycles(3);
    __bic_SR_register(SCG0); /* Enable FLL */
    while( CSCTL7 & ( FLLUNLOCK0 | FLLUNLOCK1 )); /* FLL locked */
}

static void timer2_A2_init( void )
{
    TA2CTL |= TACLR;         /* Clear TAxR */
    TA2CTL |= TASSEL__SMCLK; /* Clock source SMCLK */
    TA2CTL |= ID__8;         /* Set timer clock to 1 MHz */

    TA2CCTL0 = CCIE;         /* CCR0 interrupt enabled */

    TA2CTL |= MC_2;          /* Continuous up mode */
}

void system_init( void )
{
    WDTCTL = WDTPW | WDTHOLD;

    /* Set all GPIO to output low for power savings */
    PAOUT = 0x0000;
    PADIR = 0xFFFF;
    PBOUT = 0x0000;
    PBDIR = 0xFFFF;

    PM5CTL0 &= ~LOCKLPM5;

    clock_init();
    timer2_A2_init();

    _enable_interrupts();
}

uint64_t get_tick( void )
{
    volatile uint64_t lo_t;
    volatile uint64_t hi_t;
    uint16_t master_tmp;
    uint16_t slave_tmp;

    hi_t = (uint64_t) us_time_high;

    slave_tmp  = us_time_low;
    TA2CTL    |= MC_0;
    master_tmp = (((uint32_t)us_time_low << 16 ) | TA2R );
    TA2CTL     |= MC_2;

    if ( slave_tmp != us_time_low )
    {
        slave_tmp  = us_time_low;
        TA2CTL    |= MC_0;
        master_tmp = (((uint32_t)slave_tmp << 16 ) | TA2R );
        TA2CTL    |= MC_2;
    }

    lo_t = ((uint32_t)slave_tmp << 16) | ( master_tmp );

    return (uint64_t)(( hi_t << 32 ) | lo_t );
}

void wait( uint64_t us )
{
    uint64_t start_time = 0;
    uint64_t act_time   = 0;
    uint64_t delay      = 0;

    start_time = get_tick();

    do
    {
        act_time = get_tick();
        delay    = act_time - start_time;
    } while ( delay < us );
}

void timer2_A2_irq_hdl( void )
{
    static volatile bool_t us_time_low_overflow = FALSE;

    us_time_low++;

    if ( 0xFFFF == us_time_low )
    {
        us_time_low_overflow = TRUE;
    }

    if (( 0 == us_time_low ) && ( FALSE != us_time_low_overflow ))
    {
        us_time_high++;
        us_time_low_overflow = FALSE;
    }
}
