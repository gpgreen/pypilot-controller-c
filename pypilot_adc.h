#ifndef _PYPILOT_ADC_H_
#define _PYPILOT_ADC_H_

#include <inttypes.h>

/*-----------------------------------------------------------------------*/

extern uint8_t adc_sample_fifo_ovf(void);

/*-----------------------------------------------------------------------*/

typedef enum AdcChannel {
    Current,
    Voltage,
    ControllerTemp,
    MotorTemp,
    Rudder,
} adc_channel_t;

/*-----------------------------------------------------------------------*/

typedef enum AdcMvgAvgIdx {
    Monitor,
    Output,
} adc_mvg_avg_idx_t;

/*-----------------------------------------------------------------------*/

typedef struct AdcMvgAvg
{
    uint32_t total;
    uint16_t count;
} adc_mvg_avg_t;

extern uint32_t adc_mvgavg_calc(const adc_mvg_avg_t* avg);
extern void adc_mvgavg_reset(adc_mvg_avg_t* avg);

/*-----------------------------------------------------------------------*/

typedef struct AdcSample
{
    adc_channel_t channel;
    uint32_t total;
    uint16_t count;
} adc_sample_t;

extern void adc_sample_reset(adc_sample_t* sample, adc_channel_t channel);

/*-----------------------------------------------------------------------*/

typedef struct AdcResults
{
    adc_mvg_avg_t monitor_data[5];
    adc_mvg_avg_t output_data[5];
} adc_results_t;

extern void adc_results_init(adc_results_t* results);
extern uint8_t adc_process(adc_results_t* results);
extern uint32_t adc_get_result(adc_results_t* results, adc_channel_t channel,
                               adc_mvg_avg_idx_t n, uint16_t min_count);

/*-----------------------------------------------------------------------*/

#endif // _PYPILOT_ADC_H_
