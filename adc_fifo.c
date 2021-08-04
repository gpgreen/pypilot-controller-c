#include "adc_fifo.h"

extern inline void adc_fifo_init(struct adc_fifo* fifo);

extern inline uint8_t adc_fifo_count (const struct adc_fifo* fifo);

extern inline uint8_t adc_fifo_put_safe(struct adc_fifo* fifo, adc_sample_t* s);

extern inline int adc_fifo_put_unsafe(struct adc_fifo* fifo, adc_sample_t* s);

extern inline uint8_t adc_fifo_get_safe(struct adc_fifo* fifo, adc_sample_t* s);

extern inline int adc_fifo_get_unsafe(struct adc_fifo* fifo, adc_sample_t* s);

