#ifndef _PYPILOT_EEPROM_H_
#define _PYPILOT_EEPROM_H_

#include <inttypes.h>

/*-----------------------------------------------------------------------*/

extern uint8_t eeprom_read8(uint16_t address);

extern void eeprom_update8(uint16_t address, uint8_t val);

/*-----------------------------------------------------------------------*/

#endif // _PYPILOT_EEPROM_H_
