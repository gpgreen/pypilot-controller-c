#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include "timer.h"
#include "pypilot_fsm.h"
#include "gpio.h"
#include "crc.h"
#include "serial.h"

/*-----------------------------------------------------------------------*/

static uint8_t PENDINGPCINT16 = 0;

/*-----------------------------------------------------------------------*/

void enable_pwm(void)
{
    // set pins low
    set_ina_low();
    set_inb_low();

    // set timer to phase correct pwm, toggle OC0A, clk/64
    OCR0A = 0;
    TCCR0A = _BV(COM0A1) | _BV(WGM00);
    TCCR0B = _BV(WGM02) | _BV(CS01) | _BV(CS00);
}

/*-----------------------------------------------------------------------*/

void disable_pwm(void)
{
    // set pins low
    set_ina_low();
    set_inb_low();

    TCCR0A = 0;
    TCCR0B = 0;
}

/*-----------------------------------------------------------------------*/

void set_duty_cycle(uint16_t duty)
{
    // set duty
    if (duty > 250)
        duty = 250;
    OCR0A = (uint8_t)duty;
}

/*-----------------------------------------------------------------------*/

void position(state_t* state, uint16_t val)
{
    // set current position
    state->lastpos = val;
    uint8_t newpos;
    if (val >= 1000)
        newpos = (val - 1000) / 4;
    else
        newpos = (1000 - val) / 4;
    set_duty_cycle(newpos);
    // set pins to correct state
    if (val > 1040) {
        set_ina_low();
        set_inb_high();
    } else if (val < 960) {
        set_ina_high();
        set_inb_low();
    } else {
        set_ina_low();
        set_inb_low();
    }
}

/*-----------------------------------------------------------------------*/

void update_position(state_t* state)
{
    int16_t speed_rate = state->max_slew_speed;
    int16_t slow_rate = state->max_slew_slow;
    int16_t cmd_value = state->command_value;
    int16_t cur_value = state->lastpos;
    int16_t diff = cmd_value - cur_value;
    // limit motor speed change to within speed and slow slew rates
    if (diff > 0) {
        if (cur_value < 1000) {
            if (diff > slow_rate)
                diff = slow_rate;
        } else if (diff > speed_rate)
            diff = speed_rate;
    } else {
        if (cur_value > 1000) {
            if (diff < -slow_rate)
                diff = -slow_rate;
        } else if (diff < -speed_rate)
            diff = -speed_rate;
    }
    uint16_t new_pos = cur_value;
    if (diff < 0) {
        // check if potential pos is less 0
        if (-diff > new_pos)
            new_pos = 0;
        else
            new_pos -= -diff;
    } else if (diff > 0) {
        // check if potential pos is > 2000
        if (diff + new_pos > 2000)
            new_pos = 2000;
        else
            new_pos += diff;
    }
    position(state, new_pos);
}

/*-----------------------------------------------------------------------*/

void stop(state_t* state)
{
    position(state, 1000);
    state->command_value = 1000;
}

/*-----------------------------------------------------------------------*/

void power_down_mode(state_t* state, adc_results_t* adc_results)
{
    // delay for a bit, so that usart can finish any transmissions
    uint32_t jif = jiffie();
    while (duration(jif) < 10) ;

    // disable modules
    uint8_t spcr, ucsr;
    ADCSRA = 0;
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        spcr = SPCR;
        SPCR = 0;
        ucsr = UCSR0B;
        UCSR0B = 0;
    }

    // turn off clocks
    PRR |= _BV(PRUSART0) | _BV(PRSPI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRADC);

    // set watchdog delay longer
    wdt_enable(WDTO_2S);

    // set PCINT16 interrupt
    PCICR |= _BV(PCIE2);
    PCMSK2 |= _BV(0);

    wdt_reset();

    // do the power down, if PCINT16 interrupt hasn't happened
    // no other interrupts are important
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    if (!PENDINGPCINT16)
    {
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
    }
    sei();
    PENDINGPCINT16 = 0;

    // watchdog delay back to normal
    wdt_enable(WDTO_250MS);

    // stop PCINT16 interrupt
    PCMSK2 &= ~_BV(0);
    PCICR |= ~_BV(PCIE2);

    // turn on clocks
    PRR &= ~(_BV(PRUSART0) | _BV(PRSPI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRADC));

    // enable modules
    adc_results_init(adc_results);
    SPCR = spcr;
    UCSR0B = ucsr;
}

/*-----------------------------------------------------------------------*/

void housekeeping(state_t* state)
{
}

/*-----------------------------------------------------------------------*/

void fsm_state_send(const state_t* state)
{
    uint8_t pkt[4];
    pkt[0] = CurrentFSMCode;
    pkt[1] = state->machine_state.current;
    pkt[2] = 0;
    pkt[3] = crc8(pkt, 3);
    serial_transmit_bytes(pkt, 4);
    pkt[0] = PreviousFSMCode;
    pkt[1] = state->machine_state.previous;
    pkt[3] = crc8(pkt, 3);
    serial_transmit_bytes(pkt, 4);
}

/*-----------------------------------------------------------------------*/

void fsm_state_memo_copy(fsm_state_memo_t* new, const fsm_state_memo_t* old)
{
    new->current = old->current;
    new->previous = old->previous;
}

/*-----------------------------------------------------------------------*/

void change_state(fsm_state_memo_t* new_memo,
                  fsm_states_t new_state, const fsm_state_memo_t* memo)
{
    new_memo->current = new_state;
    switch (memo->current) {
    case Start: case Wait: case Engaged: case Operational: case Disengaged:
    case Detach: case PowerDown:
        new_memo->previous = memo->current;
        break;
    default:
        new_memo->previous = memo->previous;
        break;
    }
}

/*-----------------------------------------------------------------------*/

void fsm_process(state_t* state, packet_state_t* packet_state, adc_results_t* adc_results)
{
    int packet_received = 0;
    
    // process packet if one received
    if (state->cmd.pending_cmd == ProcessPacket)
    {
        packet_received = 1;
        packet_process(state, packet_state);
    }
    
    // do the housekeeping
    housekeeping(state);

    // check for stop
    if (state->cmd.pending_cmd == Stop)
        stop(state);

    // if packet received, send one back
    if (packet_received) {
        state->cmd.send_cmd = packet_build(
            packet_state, state->flags, state->cmd.out_packet);
    }

    fsm_state_memo_t new_state;
    fsm_state_memo_copy(&new_state, &state->machine_state);
    
    switch(state->machine_state.current) {
    case Start:
        change_state(&new_state, WaitEntry, &state->machine_state);
        break;
    case WaitEntry:
        state->timeout = 0;
        change_state(&new_state, Wait, &state->machine_state);
        break;
    case Wait:
        if (state->timeout > 30 || state->cmd.pending_cmd == Disengage)
            change_state(&new_state, DetachEntry, &state->machine_state);
        else if (state->cmd.pending_cmd == Engage)
            change_state(&new_state, Engage, &state->machine_state);
        break;
    case Engaged:
        state->timeout = 0;
        state->flags |= _BV(ENGAGED);
        enable_pwm();
        position(state, 1000);
        change_state(&new_state, Operational, &state->machine_state);
        break;
    case Operational:
        update_position(state);
        if (state->timeout > 30 || state->cmd.pending_cmd == Disengage)
            change_state(&new_state, DisengagedEntry, &state->machine_state);
        break;
    case DisengagedEntry:
        stop(state);
        state->flags &= ~_BV(ENGAGED);
        if (state->cmd.pending_cmd == Engage)
            change_state(&new_state, Engaged, &state->machine_state);
        else
            change_state(&new_state, Disengaged, &state->machine_state);
        break;
    case Disengaged:
        if (state->cmd.pending_cmd == Engage)
            change_state(&new_state, Engaged, &state->machine_state);
        else if (state->timeout > 62)
            change_state(&new_state, DetachEntry, &state->machine_state);
        break;
    case DetachEntry:
        disable_pwm();
        if (state->cmd.pending_cmd == Engage)
            change_state(&new_state, Engaged, &state->machine_state);
        else
            change_state(&new_state, Detach, &state->machine_state);
        break;
    case Detach:
        if (state->cmd.pending_cmd == Engage)
            change_state(&new_state, Engaged, &state->machine_state);
        else if (state->timeout > 62 && state->comm_timeout > 250)
            change_state(&new_state, PowerDown, &state->machine_state);
        break;
    case PowerDown:
        power_down_mode(state, adc_results);
        state->timeout = 0;
        state->comm_timeout -= 250;
        state->command_value = 1000;
        state->lastpos = 1000;
        state->flags = 0;
        state->cmd.pending_cmd = None;
        state->cmd.send_cmd = Skip;
        change_state(&new_state, WaitEntry, &state->machine_state);
        break;
    }

    // check and see if machine state has changed, if so, change previous state
    if (state->prev_state.current != state->machine_state.current
        || state->prev_state.previous != state->machine_state.previous) {
        fsm_state_memo_copy(&state->prev_state, &state->machine_state);
        fsm_state_send(state);
    }
    fsm_state_memo_copy(&state->machine_state, &new_state);
}

/*-----------------------------------------------------------------------*/

ISR(PCINT2_vect)
{
    // set flag if PD0, or RXD is on
    if (PIND & _BV(0))
        PENDINGPCINT16 = 1;
}

/*-----------------------------------------------------------------------*/
