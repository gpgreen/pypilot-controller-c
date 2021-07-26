#ifndef _PACKET_H_
#define _PACKET_H_

/*-----------------------------------------------------------------------*/
#include <inttypes.h>
#include "globals.h"

/*-----------------------------------------------------------------------*/

typedef struct packet_state 
{
    uint8_t out_sync_pos;
    uint8_t eeprom_read_addr;
    uint8_t eeprom_end_addr;
} packet_state_t;

/*-----------------------------------------------------------------------*/

extern void packet_initialize_state(packet_state_t* pkt_state);

extern void packet_process(
    state_t* state, packet_state_t* pkt_state, uint8_t* pkt);

extern outgoing_packet_cmd_t packet_build(
    packet_state_t* state, status_flags_t flags, uint8_t* pkt);

/*-----------------------------------------------------------------------*/

#endif // _PACKET_H_
