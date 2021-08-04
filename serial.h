#ifndef _SERIAL_H_
#define _SERIAL_H_

/*-----------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "globals.h"

/*-----------------------------------------------------------------------*/

typedef struct serial_state 
{
    uint8_t incoming_bytes[3];
    uint8_t incoming_idx;
    uint8_t sync_count;
} serial_state_t;

/*-----------------------------------------------------------------------*/

extern void serial_initialize_state(serial_state_t* serial_state);

extern uint8_t serial_get_next_byte(uint8_t *ch);

extern uint8_t serial_transmit_bytes(uint8_t *buf, uint8_t buflen);

extern int serial_char_send(char c, FILE* unused);

extern void serial_send(command_to_execute_t* cmd);

extern uint8_t serial_process(serial_state_t* state, uint16_t* flags, command_to_execute_t* cmd);

/*-----------------------------------------------------------------------*/

#endif // _SERIAL_H_
