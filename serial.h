#ifndef _SERIAL_H_
#define _SERIAL_H_

/*-----------------------------------------------------------------------*/
#include <inttypes.h>
#include "globals.h"

/*-----------------------------------------------------------------------*/

typedef struct serial_state 
{
    uint8_t incoming_bytes[3];
    uint8_t incoming_idx;
    uint8_t sync_count;
} serial_state_t;

/*-----------------------------------------------------------------------*/

extern uint8_t get_next_byte(uint8_t *ch);

extern uint8_t transmit_bytes(uint8_t *buf, uint8_t buflen);

extern void send_serial(command_to_execute_t* cmd);

extern uint8_t process_serial(serial_state_t* state, uint16_t* flags, command_to_execute_t* cmd);

/*-----------------------------------------------------------------------*/

#endif // _SERIAL_H_
