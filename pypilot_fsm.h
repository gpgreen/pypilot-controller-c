#ifndef _PYPILOT_FSM_H_
#define _PYPILOT_FSM_H_

#include <stdint.h>
#include "globals.h"
#include "packet.h"
#include "pypilot_adc.h"

/*-----------------------------------------------------------------------*/

extern void fsm_process(state_t* state, packet_state_t* packet_state,
                        adc_results_t* adc_results);

/*-----------------------------------------------------------------------*/

#endif // _PYPILOT_FSM_H_
