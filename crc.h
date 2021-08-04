#ifndef _CRC_H_
#define _CRC_H_

#include <stdint.h>

/* Copyright (C) 2018 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

extern uint8_t crc8_with_init(uint8_t init_value, uint8_t *pcBlock, uint8_t len);

extern uint8_t crc8(uint8_t *pcBlock, uint8_t len);

#endif // _CRC_H_
