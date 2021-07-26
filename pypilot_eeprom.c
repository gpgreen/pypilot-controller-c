#include <avr/eeprom.h>
#include "pypilot_eeprom.h"

/*-----------------------------------------------------------------------*/

uint8_t eeprom_read8(uint16_t address)
{
    // use 3 banks
    uint8_t v[3];
    for (int i=0; i<3; ++i)
    {
        uint16_t addr = i * 256 + address;
        v[i] = eeprom_read_byte((const uint8_t*)addr);
    }
    if (v[1] == v[2]) {
        return v[2];
    } else {
        return v[0];
    }
}

/*-----------------------------------------------------------------------*/

void eeprom_update8(uint16_t address, uint8_t val)
{
    // write in 3 locations
    for (int i = 0; i<3; ++i) {
        uint16_t addr = i * 256 + address;
        eeprom_update_byte((uint8_t*)addr, val);
    }
}
