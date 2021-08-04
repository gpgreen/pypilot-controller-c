#include <stdint.h>
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

inline void set_ina_low(void)
{
    PORT_INA &= ~_BV(P_INA);
}

inline void set_inb_low(void)
{
    PORT_INB &= ~_BV(P_INB);
}

inline void set_ina_high(void)
{
    PORT_INA |= _BV(P_INA);
}

inline void set_inb_high(void)
{
    PORT_INB |= _BV(P_INB);
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

    // set INA/B pins to output
    DDR_INA |= _BV(P_INA);
    DDR_INB |= _BV(P_INB);

    // set INA/B pins low
    set_ina_low();
    set_ina_low();

    // set SPI pins
    DDR_CANCS |= P_CANCS;
    DDR_SPI |= P_MISO;
    // set CS high
    PORT_CANCS |= P_CANCS;
}
