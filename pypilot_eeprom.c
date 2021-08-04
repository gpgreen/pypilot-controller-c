#include <avr/eeprom.h>
#include "pypilot_eeprom.h"
#include "watchdog.h"

/*-----------------------------------------------------------------------*/

uint8_t eeprom_read8(uint8_t address)
{
    // use 3 banks
    uint8_t v[3];
    for (int i=0; i<3; ++i)
    {
        uint16_t addr = i * 256 + address + sizeof(struct reset_count);
        uint16_t base_addr = addr & 0xFFFE;
        uint16_t word = eeprom_read_word((const uint16_t*)base_addr);
        v[i] = base_addr != addr ? (uint8_t)(word >> 8) : (uint8_t)(word & 0xFF);
    }
    if (v[1] == v[2]) {
        return v[2];
    } else {
        return v[0];
    }
}

/*-----------------------------------------------------------------------*/

void eeprom_update8(uint8_t address, uint8_t val)
{
    // write in 3 locations
    for (int i = 0; i<3; ++i) {
        uint16_t addr = i * 256 + address + sizeof(struct reset_count);
        uint16_t base_addr = addr & 0xFFFE;
        uint16_t cur_val = eeprom_read_word((const uint16_t*)base_addr);
        uint16_t new_val;
        if (base_addr != addr) {
            // odd address case
            new_val = (cur_val & 0xFF00) | val;
        } else {
            // even address case
            new_val = (val >> 8) | (cur_val & 0xFF);
        }
        eeprom_update_word((uint16_t*)base_addr, new_val);
    }
}
