#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <inttypes.h>
#include "can.h"

// the can bus state
extern canbus_state_t g_state;

// the cycle time (approx 80hz) in tenth milliseconds
extern uint32_t g_cycle_time;

// error number
extern volatile uint8_t errcode;

// firmware failed
extern void panic(void);

#endif  // GLOBALS_H_
