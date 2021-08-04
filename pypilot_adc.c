#include <avr/io.h>
#include "pypilot_adc.h"
#include "globals.h"
#include "adc_fifo.h"

/*-----------------------------------------------------------------------*/

static uint8_t SAMPLE_OVF = 0;

static struct adc_fifo SAMPLE_QUEUE;

static uint8_t CHANNELS_ENABLED = 0x3;

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
    ADMUX |= _BV(REFS0) | _BV(MUX0);
    // disable digital inputs
    DIDR0 = 0x3F;
    // start conversion
    ADCSRA |= _BV(ADSC);
    memset(results->monitor_data, 0, sizeof(adc_mvg_avg_t) * 5);
    memset(results->output_data, 0, sizeof(adc_mvg_avg_t) * 5);
}

/*-----------------------------------------------------------------------*/

// add the sample to a channel
void adc_add_sample(adc_results_t* results, const adc_sample_t* sample)
{
    adc_mvg_avg_t *avg = &results->monitor_data[sample->channel];
    uint32_t allowed_total = UINT32_MAX - avg->total;
    uint16_t allowed_count = UINT16_MAX - avg->count;
    if (allowed_total > sample->total && allowed_count > sample->count) 
    {
        avg->count += sample->count;
        avg->total += sample->total;
    }
    avg = &results->output_data[sample->channel];
    allowed_total = UINT32_MAX - avg->total;
    allowed_count = UINT16_MAX - avg->count;
    if (allowed_total > sample->total && allowed_count > sample->count) 
    {
        avg->count += sample->count;
        avg->total += sample->total;
    }
}

/*-----------------------------------------------------------------------*/

// check the sample queue for sample and add them to the moving averages
uint8_t adc_process(adc_results_t* results)
{
    // if sample queue has overflow'ed, exit
    if (adc_sample_fifo_ovf())
        return 0xFF;
    // check for new sample
    adc_sample_t sample;
    for (int i=0; i<10; ++i)
    {
        int res = adc_fifo_get_safe(&SAMPLE_QUEUE, &sample);
        if (res == ADC_FIFO_OK)
            adc_add_sample(results, &sample);
        else
            break;
    }
    return 0;
}

/*-----------------------------------------------------------------------*/

uint32_t adc_get_result(
    adc_results_t* results, adc_channel_t channel,
    adc_mvg_avg_idx_t n, uint16_t min_count)
{
    uint32_t result = 0;
    adc_mvg_avg_t* avg = (n == Monitor) ?
        &results->monitor_data[channel]
        : &results->output_data[channel];
    if (avg->count >= min_count) {
        result = adc_mvgavg_calc(avg);
        adc_mvgavg_reset(avg);
    }
    return result;
}

/*-----------------------------------------------------------------------*/

uint8_t adc_channel_enabled(adc_channel_t ch)
{
    // see if channel is enabled
    uint8_t enabled = 0;
    switch (ch) {
    case Current:
        if ((CHANNELS_ENABLED & 0x1) == 0x1)
            enabled = 0xFF;
        break;
    case Voltage:
        if ((CHANNELS_ENABLED & 0x2) == 0x2)
            enabled = 0xFF;
        break;
    case ControllerTemp:
        if ((CHANNELS_ENABLED & 0x4) == 0x4)
            enabled = 0xFF;
        break;
    case MotorTemp:
        if ((CHANNELS_ENABLED & 0x8) == 0x8)
            enabled = 0xFF;
        break;
    case Rudder:
        if ((CHANNELS_ENABLED & 0x10) == 0x10)
            enabled = 0xFF;
        break;
    }
    return enabled;
}

/*-----------------------------------------------------------------------*/

ISR(ADC_vect)
{
    static adc_sample_t SAMPLE = {
        .channel=Current,
        .total=0,
        .count=0};
    static uint8_t CNT = 0;

    // throw away first 3 samples
    if (CNT == 3) {
        SAMPLE.total += ADC;
        SAMPLE.count++;
        uint8_t switch_channel = 0;
        switch (SAMPLE.channel) {
        case Current:
            if (SAMPLE.count == 50)
                switch_channel = 0xFF;
            break;
        case Voltage:
            if (SAMPLE.count == 8)
                switch_channel = 0xFF;
            break;
        case ControllerTemp: case MotorTemp: case Rudder:
            if (SAMPLE.count == 1)
                switch_channel = 0xFF;
            break;
        }
        if (switch_channel) {
            uint8_t enabled = adc_channel_enabled(SAMPLE.channel);
            // if enabled, queue the sample
            if (enabled && adc_fifo_put_unsafe(&SAMPLE_QUEUE, &SAMPLE) == ADC_FIFO_FULL) {
                SAMPLE_OVF = 0xFF;
            }
            // get next channel
            adc_channel_t ch = SAMPLE.channel;
            do {
                if (++ch == 5)
                    ch = 0;
            } while (adc_channel_enabled(ch) == 0);
            // change mux
            switch (ch) {
            case Current:
                // Current is on ADC1
                ADMUX = _BV(REFS0) | _BV(MUX0);
                break;
            case Voltage:
                // Voltage on pin ADC0
                ADMUX = _BV(REFS0);
                break;
            case ControllerTemp:
                // Controller Temp on pin ADC2
                ADMUX = _BV(REFS0) | _BV(MUX1);
                break;
            case MotorTemp:
                // Motor Temp on pin ADC3
                ADMUX = _BV(REFS0) | _BV(MUX1) | _BV(MUX0);
                break;
            case Rudder:
                // Rudder angle on pin ADC4
                ADMUX = _BV(REFS0) | _BV(MUX2);
                break;
            }
            adc_sample_reset(&SAMPLE, ch);
            CNT = 0;
        }
    } else {
        CNT++;
    }
    // start a new sample
    ADCSRA |= _BV(ADSC);
}

/*-----------------------------------------------------------------------*/
