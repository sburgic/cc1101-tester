#include "rf_talk.h"

#include "cc1101.h"

static uint8_t rft_src_addr = 1;

void rft_send_ack( uint8_t destination )
{
    uint8_t out[RF_TALK_ACK_SIZE] = {0};
    uint8_t crc;

    out[0] = RF_TALK_ACK_SIZE;
    out[1] = destination;
    out[2] = rft_src_addr;
    out[3] = 0;

    crc = cc_crc( out );

    out[4] = 256 - crc;

    cc_send( out );
}
