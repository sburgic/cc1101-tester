#ifndef __RF_TALK_H__
#define __RF_TALK_H__

#include <stdint.h>

#define RF_TALK_ACK_SIZE (5)

void rft_send_ack( uint8_t destination );

#endif /* __RF_TALK_H__ */
