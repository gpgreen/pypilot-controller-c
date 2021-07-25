#ifndef GPIO_H_
#define GPIO_H_

#include "defs.h"

extern void led_on(int n);
extern void led_off(int n);
extern void led_toggle(int n);

extern void gpio_setup(void);

#endif	// GPIO_H_
