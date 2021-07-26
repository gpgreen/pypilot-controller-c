#include <avr/io.h>
#include "packet.h"
#include "pypilot_eeprom.h"

/*-----------------------------------------------------------------------*/

void packet_initialize_state(packet_state_t* pkt_state)
{
    pkt_state->out_sync_pos = 0;
    pkt_state->eeprom_read_addr = pkt_state->eeprom_end_addr = 0;
}

/*-----------------------------------------------------------------------*/

void packet_process(state_t* state, packet_state_t* pkt_state, uint8_t* pkt)
{
    uint16_t val = pkt[1] | (pkt[2] << 8);
    uint16_t max_max_current = (LOWCURRENT == 0) ? 5000 : 2000;
    switch (pkt[0]) {
    case CommandCode:
        state->timeout = 0;
        // check for valid range and none of the faults
        if (val <= 2000 &&
            (state->flags &
             (_BV(OVERTEMPFAULT) | _BV(OVERCURRENTFAULT) | _BV(BADVOLTAGEFAULT))) == 0) {
            // if rudder faults, then stop
            if ((state->flags & (_BV(PORTPINFAULT) | _BV(MAXRUDDERFAULT))) != 0
                || ((state->flags & (_BV(STBDPINFAULT) | _BV(MINRUDDERFAULT))) != 0
                    && val < 1000)) {
                state->cmd.pending_cmd = Stop;
            } else {
                // otherwise send motor the new value
                state->command_value = val;
                state->cmd.pending_cmd = Engage;
            }
        }
        break;
    case MaxCurrentCode:
        state->max_current = (val > max_max_current) ? max_max_current : val;
        break;
    case MaxControllerTempCode:
        if (val > 10000)
            val = 10000;
        state->max_controller_temp = val;
        break;
    case MaxMotorTempCode:
        if (val > 10000)
            val = 10000;
        state->max_motor_temp = val;
        break;
    case RudderMinCode:
        state->rudder_min = val;
        break;
    case RudderMaxCode:
        state->rudder_max = val;
        break;
    case DisengageCode:
        state->cmd.pending_cmd = Disengage;
        break;
    case MaxSlewCode:
        state->max_slew_speed = pkt[1];
        state->max_slew_slow = pkt[2];
        // if set at the end of the range (up to 255) no slew limit
        if (state->max_slew_speed > 250) {
            state->max_slew_speed = 250;
        }
        if (state->max_slew_slow > 250) {
            state->max_slew_slow = 250;
        }
        // must have some slew
        if (state->max_slew_speed == 0) {
            state->max_slew_speed = 1;
        }
        if (state->max_slew_slow == 0) {
            state->max_slew_slow = 1;
        }
        break;
    case EEPROMReadCode:
        if (pkt_state->eeprom_read_addr == pkt_state->eeprom_end_addr) {
            pkt_state->eeprom_read_addr = pkt[1];
            pkt_state->eeprom_end_addr = pkt[2];
        }
        break;
    case EEPROMWriteCode:
        eeprom_update8(pkt[1], pkt[2]);
        break;
    case ReprogramCode:
        break;
    case ResetCode:
        state->flags &= ~_BV(OVERCURRENTFAULT);
        break;
    case RudderRangeCode:
        break;
    default:
        break;
    }
}

/*-----------------------------------------------------------------------*/

void build_current_pkt(
    packet_state_t* pkt_state, packet_type_t* pkt_ty, uint16_t* val)
{
}

/*-----------------------------------------------------------------------*/

outgoing_packet_cmd_t packet_build(
    packet_state_t* pkt_state, status_flags_t flags, uint8_t* pkt)
{
    packet_type_t pkt_ty = InvalidCode;
    uint16_t value = 0;
    uint8_t skip_out_sync = 0;

    switch (pkt_state->out_sync_pos) {
    case 0: case 10: case 20: case 30:
        value = flags;
        pkt_ty = FlagsCode;
        break;
    case 1: case 4: case 7: case 11: case 14: case 17: case 21: case 24:
    case 27: case 31: case 34: case 37: case 40:
        build_current_pkt(pkt_state, &pkt_ty, &value);
        break;
    case 3: case 13: case 23: case 33:
        //build_voltage_pkt(pkt_state, &pkt_ty, &value);
        break;
    case 2: case 5: case 8: case 12: case 15: case 18: case 22: case 25:
    case 28: case 32: case 35: case 38: case 41:
        //build_rudder_pkt(pkt_state, &pkt_ty, &value);
        break;
    case 6:
        //build_controller_temp_pkt(pkt_state, &pkt_ty, &value);
        break;
    case 9:
        //build_motor_temp_pkt(pkt_state, &pkt_ty, &value);
        break;
    case 16: case 26: case 36:
        if (pkt_state->eeprom_read_addr != pkt_state->eeprom_end_addr) {
            value = eeprom_read8(pkt_state->eeprom_read_addr) << 8 | pkt_state->eeprom_read_addr;
            skip_out_sync = 1;
            // increment address
            pkt_state->eeprom_read_addr++;
            pkt_ty = EEPROMValueCode;
        }
        break;
    default:
        break;
    }
    // advance cycle if it is not suppressed
    if (skip_out_sync == 0) {
        pkt_state->out_sync_pos++;
        if (pkt_state->out_sync_pos == 42)
            pkt_state->out_sync_pos = 0;
    }
    // check if outgoing packet is ready to send
    if (pkt_ty != InvalidCode) {
        pkt[0] = pkt_ty;
        pkt[1] = value & 0xFF;
        pkt[2] = value >> 8;
        return Send;
    }
    return Skip;
}

/*-----------------------------------------------------------------------*/


