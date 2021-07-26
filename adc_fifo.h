/*------------------------------------------------*/
/* FIFO functions                                 */
#ifndef _ADC_FIFO_H_
#define _ADC_FIFO_H_

#include "defs.h"
#include <string.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "pypilot_adc.h"

// Return values
#define ADC_FIFO_OK 0
#define ADC_FIFO_EMPTY -1
#define ADC_FIFO_FULL -2

#define ADC_SAMPLE_FIFO_SIZE 64

struct adc_fifo {
	volatile uint8_t idx_w;
	volatile uint8_t idx_r;
	volatile uint8_t count;
	adc_sample_t buf[ADC_SAMPLE_FIFO_SIZE];
};

/* fifo_init
 * call if needed to initialize fifo
 */
inline void adc_fifo_init(struct adc_fifo* fifo)
{
 	fifo->idx_r = 0;
	fifo->idx_w = 0;
	fifo->count = 0;
    memset(fifo->buf, 0, sizeof(adc_sample_t) * ADC_SAMPLE_FIFO_SIZE);
}

/* 
 * fifo_count
 * returns number of bytes in the fifo
 */
inline uint8_t adc_fifo_count (const struct adc_fifo* fifo)
{
	return fifo->count;
}

/*
 * fifo_put_safe_unblocking
 * place a byte in the fifo. Returns whether fifo was full or not
 * Safe to use with only 1 concurrent reader or writer.
 */
inline uint8_t adc_fifo_put_safe(struct adc_fifo* fifo, adc_sample_t* s)
{
    uint8_t retval = ADC_FIFO_FULL;
	ATOMIC_BLOCK(ATOMIC_FORCEON) 
	{
        if (fifo->count < ADC_SAMPLE_FIFO_SIZE) {
            fifo->buf[fifo->idx_w] = *s;
            ++fifo->count;
            if (++fifo->idx_w >= ADC_SAMPLE_FIFO_SIZE)
                fifo->idx_w = 0;
            retval = ADC_FIFO_OK;
        }
	}
    return retval;
}

/*
 * fifo_put_unsafe
 * place a byte in the fifo. 
 * Intended to be used when interrupts are already disabled
 * Returns FIFO_FULL if no space left
 * Returns FIFO_OK when arg byte is copied to FIFO
 */
inline int adc_fifo_put_unsafe(struct adc_fifo* fifo, adc_sample_t* s)
{
	uint8_t i;

	i = fifo->idx_w;
	if (fifo->count >= ADC_SAMPLE_FIFO_SIZE)
		return ADC_FIFO_FULL;
 	fifo->buf[i++] = *s;
	++fifo->count;
	if (i >= ADC_SAMPLE_FIFO_SIZE)
		i = 0;
	fifo->idx_w = i;
	return ADC_FIFO_OK;
}

/*
 * fifo_get_safe_unblocking
 * get a byte from the fifo
 * safe to use with only 1 concurrent reader and writer.
 * Returns true if byte retrieved, false if not
 */
inline uint8_t adc_fifo_get_safe(struct adc_fifo* fifo, adc_sample_t* s)
{
    uint8_t retval = ADC_FIFO_FULL;
	ATOMIC_BLOCK(ATOMIC_FORCEON) 
	{
        if (fifo->count != 0)
        {
            *s = fifo->buf[fifo->idx_r];
            --fifo->count;
            if (++fifo->idx_r >= ADC_SAMPLE_FIFO_SIZE)
                fifo->idx_r = 0;
            retval = ADC_FIFO_OK;
        }
	}
	return retval;
}

/*
 * fifo_get_unsafe
 * get a byte from the fifo
 * Intended to be used when interrupts are already disabled
 * Returns FIFO_EMPTY if fifo has no bytes,
 * Returns FIFO_OK and byte is copied to byte arg
 */
inline int adc_fifo_get_unsafe(struct adc_fifo* fifo, adc_sample_t* s)
{
	uint8_t i;

	i = fifo->idx_r;
	if (fifo->count == 0)
		return ADC_FIFO_EMPTY;
	*s = fifo->buf[i++];
	--fifo->count;
	if (i >= ADC_SAMPLE_FIFO_SIZE)
		i = 0;
	fifo->idx_r = i;

	return ADC_FIFO_OK;
}

#endif // _ADC_FIFO_H_
