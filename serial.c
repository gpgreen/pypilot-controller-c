/*-----------------------------------------------------------------------*/
#include <string.h>
#include <avr/sfr_defs.h>
#include "uart.h"
#include "serial.h"
#include "crc.h"

/*-----------------------------------------------------------------------*/
void serial_initialize_state(serial_state_t* serial_state)
{
    memset(serial_state->incoming_bytes, 0, 3);
    serial_state->incoming_idx = 0;
    serial_state->sync_count = 0;
}

/*-----------------------------------------------------------------------*/
// returns 0 if no new byte, 1 if new byte, byte is placed in ch
uint8_t serial_get_next_byte(uint8_t *ch)
{
    if (uart_getchar_unblocking(ch))
        return 0xFF;
    return 0;
}

/*-----------------------------------------------------------------------*/
// returns number of bytes placed on transmit queue
uint8_t serial_transmit_bytes(uint8_t *buf, uint8_t buflen)
{
    uint8_t count = 0;
    for (int i=0; i<buflen; i++)
    {
        if (uart_putchar_unblocking(buf[i]))
            count++;
        else
            break;
    }
    return count;
}

/*-----------------------------------------------------------------------*/
/* check for incoming bytes, convert to packets by checking crc
 * if a packet is received, set pending_cmd to process it
 * set flags as required for communication state
 */
void serial_send(command_to_execute_t* cmd)
{
    if (cmd->send_cmd == Send)
    {
        uint8_t crc = crc8(cmd->out_packet, 3);
        uint8_t pkt[4] = {cmd->out_packet[0], cmd->out_packet[1], cmd->out_packet[2], crc};
        serial_transmit_bytes(pkt, 4);
    }
}


/*-----------------------------------------------------------------------*/
/* check for incoming bytes, convert to packets by checking crc
 * if a packet is received, set pending_cmd to process it
 * set flags as required for communication state
 * use in stdout, stderr file streams
 */
int serial_char_send(char c, FILE* unused)
{
    static uint8_t msg_buf[4] = {MsgCode, 0, 0, 0};
    static uint8_t msg_idx = 1;

    msg_buf[msg_idx] = c;
    if (++msg_idx == 3) {
        goto send_pkt;
    } else if (c == '\n') {
        if (msg_idx == 2)
            msg_buf[2] = 0;
        goto send_pkt;
    }
    return 0;
send_pkt:
    msg_buf[3] = crc8(msg_buf, 3);
    msg_idx = 1;
    serial_transmit_bytes(msg_buf, 4);
    return 0;
}


/*-----------------------------------------------------------------------*/
/* check for incoming bytes, convert to packets by checking crc
 * if a packet is received, set pending_cmd to process it
 * set flags as required for communication state
 */
uint8_t serial_process(serial_state_t* state, status_flags_t* flags, command_to_execute_t* cmd)
{
    // check overflow flags
    if (uart_read_fifo_ovf()) {
        *flags |= _BV(READQUEUEOVF);
        panic();
    }
    if (uart_write_fifo_ovf()) {
        *flags |= _BV(WRITEQUEUEOVF);
        panic();
    }
    uint8_t reset_comm_timeout = 0;
    uint8_t byte;
    if (serial_get_next_byte(&byte)) {
        // reset the serial timeout now, since we've received data
        reset_comm_timeout = 0xFF;
        if (state->incoming_idx < 3) {
            state->incoming_bytes[state->incoming_idx] = byte;
            state->incoming_idx++;
        } else {
            uint8_t pkt_crc = crc8(state->incoming_bytes, 3);
            if (byte == pkt_crc) {
                // valid packet
                state->incoming_idx = 0;
                if (state->sync_count >= 2) {
                    cmd->pending_cmd = ProcessPacket;
                    memcpy(cmd->in_packet, state->incoming_bytes, 3);
                } else {
                    state->sync_count++;
                }
                *flags |= _BV(SYNC);
                *flags &= ~_BV(INVALID);
            } else {
                // invalid packet
                state->sync_count = 0;
                state->incoming_bytes[0] = state->incoming_bytes[1];
                state->incoming_bytes[1] = state->incoming_bytes[2];
                state->incoming_bytes[2] = byte;
                *flags &= ~_BV(SYNC);
                *flags |= _BV(INVALID);
                cmd->pending_cmd = Stop;
            }
        }
    }
    return reset_comm_timeout;
}
/*-----------------------------------------------------------------------*/

