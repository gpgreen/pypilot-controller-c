#ifndef GPIO_H_
#define GPIO_H_

#include "defs.h"

extern void led_on(int n);
extern void led_off(int n);
extern void led_toggle(int n);

extern void set_ina_low(void);
extern void set_inb_low(void);
extern void set_ina_high(void);
extern void set_inb_high(void);

extern void gpio_setup(void);

#endif	// GPIO_H_
