#include <avr/io.h>
#include "pypilot_adc.h"
#include "globals.h"
#include "adc_fifo.h"

/*-----------------------------------------------------------------------*/

static uint8_t SAMPLE_OVF = 0;

static struct adc_fifo SAMPLE_QUEUE;

static uint8_t CHANNELS_ENABLED = 0;

/*-----------------------------------------------------------------------*/

uint8_t adc_sample_fifo_ovf(void)
{
    return SAMPLE_OVF;
}

/*-----------------------------------------------------------------------*/

uint32_t adc_mvgavg_calc(const adc_mvg_avg_t* avg)
{
    if (avg->count > 0)
        return avg->total / avg->count;
    else
        return 0;
}

/*-----------------------------------------------------------------------*/

void adc_mvgavg_reset(adc_mvg_avg_t* avg)
{
    avg->count = 0;
    avg->total = 0;
}

/*-----------------------------------------------------------------------*/

void adc_sample_reset(adc_sample_t* sample, adc_channel_t channel)
{
    sample->channel = channel;
    sample->total = 0;
    sample->count = 0;
}

/*-----------------------------------------------------------------------*/

void adc_results_init(adc_results_t* results)
{
    SAMPLE_OVF = 0;
    adc_fifo_init(&SAMPLE_QUEUE);
    
    CHANNELS_ENABLED = 0x3;
    // given defines, we have more channels enabled
    
    ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    ADMUX |= _BV(REFS0) | _BV(MUX1);
    // disable digital inputs
    DIDR0 = 0x3F;
    // start conversion
    ADCSRA |= _BV(ADSC);
}

/*-----------------------------------------------------------------------*/

uint8_t adc_process(adc_results_t* results)
{
    return 0;
}

/*-----------------------------------------------------------------------*/

uint32_t adc_get_result(
    adc_results_t* results, adc_channel_t channel,
    adc_mvg_avg_idx_t n, uint16_t min_count)
{
    return 0;
}

/*-----------------------------------------------------------------------*/

ISR(ADC_vect)
{
}

/*-----------------------------------------------------------------------*/
