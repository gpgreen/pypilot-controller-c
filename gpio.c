#include <inttypes.h>
#include <avr/io.h>

#include "gpio.h"
#include "globals.h"

void led_on(int n) 
{
    switch (n) {
    case 0:
        PORT_SPI |= _BV(P_SCK);
        break;
    case 1:
        PORTB |= _BV(0);
        break;
    case 2:
        PORTD |= _BV(7);
        break;
    case 3:
        PORTD |= _BV(5);
        break;
    case 4:
        PORTD |= _BV(4);
        break;
    case 5:
        PORTD |= _BV(3);
        break;
    case 6:
        PORTD |= _BV(2);
        break;
    default:
        panic();
    }
}

void led_off(int n)
{
    switch (n) {
    case 0:
        PORT_SPI &= ~_BV(P_SCK);
        break;
    case 1:
        PORTB &= ~_BV(0);
        break;
    case 2:
        PORTD &= ~_BV(7);
        break;
    case 3:
        PORTD &= ~_BV(5);
        break;
    case 4:
        PORTD &= ~_BV(4);
        break;
    case 5:
        PORTD &= ~_BV(3);
        break;
    case 6:
        PORTD &= ~_BV(2);
        break;
    default:
        panic();
    }
}

void led_toggle(int n) 
{
    switch (n) {
    case 0:
        PORT_SPI ^= _BV(P_SCK);
        break;
    case 1:
        PORTB ^= _BV(0);
        break;
    case 2:
        PORTD ^= _BV(7);
        break;
    case 3:
        PORTD ^= _BV(5);
        break;
    case 4:
        PORTD ^= _BV(4);
        break;
    case 5:
        PORTD ^= _BV(3);
        break;
    case 6:
        PORTD ^= _BV(2);
        break;
    default:
        panic();
    }
}

void gpio_setup(void)
{
    // for debugging, we have 7 led's, 1 of which is tied to SPI SCK pin
    DDRB |= _BV(0);
    DDRD |= _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(7);

    // pwm pin d6 of arduino, make it output
    DDRD |= _BV(6);

    // disconnect all digital registers from adc pins
    DIDR0 |= 0x3F;
}
