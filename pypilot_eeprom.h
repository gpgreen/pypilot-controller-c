#ifndef _PYPILOT_EEPROM_H_
#define _PYPILOT_EEPROM_H_

#include <stdint.h>

/*-----------------------------------------------------------------------*/

extern uint8_t eeprom_read8(uint8_t address);

extern void eeprom_update8(uint8_t address, uint8_t val);

/*-----------------------------------------------------------------------*/

#endif // _PYPILOT_EEPROM_H_
