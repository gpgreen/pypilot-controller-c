/*
 * note that the device specific header files are located
 * at: /usr/lib/avr/include/avr/
 *
 * Fuse bits for atmega328p
 * internal Crystal Oscillator 6CK, start up + 65ms
 * clock prescaler divide by 8
 * boot flash section size=1024 words, boot start address=$3C00, BOOTRST=01
 * serial programming enabled
 * brown-out at 4.3V
 * Low=0x62 Hi=0xdb Ext=0xfc
 * avrdude settings:
 * -U lfuse:w:0x62:m -U hfuse:w:0xdb:m
 * -U efuse:w:0xfc:m
 * from http://www.engbedded.com/fusecalc/
 *
 * Timer0 is used for pwm
 * Timer1 is used for 20/80 hz timer
 * Timer2 is used for timer.h (needs define in defs.h)
 */

#include "defs.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "globals.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "timer1.h"
#include "spi.h"
#include "can.h"
#include "watchdog.h"
#include "mcp2515.h"
#include "serial.h"
#include "packet.h"
#include "pypilot_adc.h"

/*-----------------------------------------------------------------------*/
// globals

// the state
canbus_state_t g_state;

// global error code
volatile uint8_t errcode;

// the cycle time (approx 80hz) in tenth milliseconds
uint32_t g_cycle_time;

const uint8_t LOWCURRENT = 1;

/*-----------------------------------------------------------------------*/

// set serial port to stdio
static FILE uart_ostr = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
static FILE uart_istr = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

// internal timer flags

// 20 hz timer flag
volatile uint8_t g_timer20_set;
const uint8_t k_timer20_compare_count = 10;

// 200 hz timer flag
volatile uint8_t g_timer200_set;

/*-----------------------------------------------------------------------*/

// uart buffers
uint8_t tx_fifo_buffer[TX_FIFO_BUFFER_SIZE];
uint8_t rx_fifo_buffer[RX_FIFO_BUFFER_SIZE];

/*-----------------------------------------------------------------------*/

void timer1_compareA(void)
{
	g_timer200_set = 1;
    static uint8_t cnt = 0;
    if (cnt++ == k_timer20_compare_count) {
        g_timer20_set = 1;
        cnt = 0;
    }
}

static timer1_init_t timer1_settings = {
    .scale = CLK8,
    .compareA_cb = timer1_compareA,
    .compareB_cb = 0,
    // compare A triggers at 200Hz
    .compareA_val = (F_CPU / 200 / 8),
    .compareB_val = 0,
};

/*-----------------------------------------------------------------------*/

// the can stack initialization struct
can_init_t can_settings = {
    .speed_setting = CAN_250KBPS,
    .loopback_on = 0,
    .tx_wait_ms = 20,
};

/* struct for mcp2515 device specific data */
struct mcp2515_dev g_mcp2515_dev = {
	// settings structure
	.settings = &can_settings,
	// interrupt control register, controlling edge detection, or which pin change mask register
	.int_dir_reg = &PCMSK0,
	// mask for interrupt control register for edge detection, falling edge, or which pin change enable mask
	.int_dir_mask = _BV(1),
	// interrupt control register, enable/disable
	.int_en_reg = &PCICR,
	// mask for interrupt control register
	.int_en_mask = _BV(PCIE0),
	// GPIO port interrupt is on
	.port = &PORTB,
	// GPIO port direction control register
	.ddr_port = &DDRB,
    // port pin register for interrupt
    .pin = &PINB,
	// GPIO pin mask
	.port_pin = _BV(1),
};

/*-----------------------------------------------------------------------*/

static void canbus_can_error(struct can_device* dev, const can_error_t* err)
{
	printf_P(PSTR("E%02x%02x%02x\n"), (uint8_t)err->error_code,
			 (uint8_t)err->dev_buffer,
			 (uint8_t)err->dev_code);
	if (err->error_code == CAN_BUS_OFF)
		g_state = BUS_OFF;
	else if (err->error_code == CAN_BUS_PASSIVE)
		g_state = ERROR_PASSIVE;
}

/*-----------------------------------------------------------------------*/

// the can device
struct can_device candev = {
    .priv_dev = &g_mcp2515_dev,
    .devno = 1,
    .init_fn = mcp2515_init,
    .reinit_fn = mcp2515_reinit,
    .self_test_fn = mcp2515_self_test,
    .check_receive_fn = mcp2515_check_receive,
    .free_send_buffer_fn = mcp2515_get_next_free_tx_buf,
    .write_msg_fn = mcp2515_write_msg,
    .read_msg_fn = mcp2515_read_msg,
    .device_command = 0,
    .handle_int_fn = mcp2515_handle_interrupt,
    .handle_error_fn = canbus_can_error,
    .error_counts = mcp2515_error_counts,
    .clear_tx_buffers = mcp2515_clear_tx_buffers,
};

/*-----------------------------------------------------------------------*/

// panic the firmware
// blinks 4 times at 10 hz then a long pause then repeat
// the led is normally part of SPI (SCK)
void
panic(void)
{
    uint8_t count = 0;
    uint8_t major_count = 0;
    uint8_t minor_count = 0;
    // turn off spi, so led will work
    SPCR = 0;
    DDR_SPI |= _BV(P_SCK);
    led_on(0);
	while (1) {
        // reset the watchdog, so it doesn't reboot the mcu
        wdt_reset();
		// 20 hz timer
		if (g_timer20_set)
		{
            if (++count == 4) {
                count = 0;
                if (major_count == 0) {
                    led_toggle(0);
                    if (++minor_count == 7)
                        major_count++;
                } else if (major_count++ == 4) {
                    major_count = 0;
                    minor_count = 0;
                    led_on(0);
                }
            }
            g_timer20_set = 0;
		}
	}
}

/*-----------------------------------------------------------------------*/

void state_initialize(state_t* state)
{
    memset(state, 0, sizeof(state_t));
    state->machine_state[0] = state->machine_state[1] = Start;
    state->prev_state[0] = state->prev_state[1] = Start;
    state->comm_timeout = 250;
    state->command_value = state->lastpos = 1000;
    state->max_slew_speed = 15;
    state->max_slew_slow = 35;
    state->max_voltage = 1600;
    state->max_current = 2000;
    state->max_controller_temp = 7000;
    state->max_motor_temp = 7000;
    state->rudder_max = 65535;
    state->flags = 0;
    state->cmd.pending_cmd = None;
    state->cmd.send_cmd = Skip;
}

/*-----------------------------------------------------------------------*/

void
system_start(void)
{
    // first set the clock prescaler change enable
	CLKPR = _BV(CLKPCE);
	// now set the clock prescaler to clk / 2
	CLKPR = _BV(CLKPS0);

	watchdog_init(WDTO_1S);
}

/*-----------------------------------------------------------------------*/

void
ioinit(adc_results_t* adc_results)
{
	gpio_setup();

	watchdog_reset_count_update();
	
	timer_init();

    timer1_init(&timer1_settings);

	// setup the serial hardware
	uart_init(TX_FIFO_BUFFER_SIZE, tx_fifo_buffer,
              RX_FIFO_BUFFER_SIZE, rx_fifo_buffer);
	
	puts_P(PSTR("PYPILOT CONTROLLER"));
	printf_P(PSTR("Hardware: %d Software: %d.%d\n-------------------------\n"),
			 HARDWARE_REVISION, APP_VERSION_MAJOR, APP_VERSION_MINOR);
	
	// spi needs to be setup first
	if (spi_init(4) == SPI_FAILED)
		panic();
	
	puts_P(PSTR("spi initialized."));
	
	// setup CAN stack
	errcode = can_init(&can_settings, &candev);
	if (errcode == CAN_OK)
		panic();
	puts_P(PSTR("can initialized."));
	
	// self test the CAN stack
	errcode = can_self_test(&candev);
	if (errcode != CAN_OK)
		panic();
	puts_P(PSTR("can self-test complete."));

    // the adc results
    adc_results_init(adc_results);
    puts_P(PSTR("adc initialized."));
    
	watchdog_reset();
	
	watchdog_print_flags();
	
	puts_P(PSTR("ioinit complete."));

}

/*-----------------------------------------------------------------------*/

int
main(void)
{
	system_start();

	// stdout is the uart
	stdout = &uart_ostr;
	// stdin is the uart
	stdin = &uart_istr;
	
    // the state variable
    state_t state;
    state_initialize(&state);
    
    // the serial state variable
    serial_state_t serial_state;
    serial_initialize_state(&serial_state);
    
    // the packet state variable
    packet_state_t packet_state;
    packet_initialize_state(&packet_state);

    // the adc results variable
    adc_results_t adc_results;
    
    // initialize hardware
    ioinit(&adc_results);

	sei();

	// keep track of the last time
	uint32_t lt = jiffie();
	
    while(1)
    {
		watchdog_reset();

        adc_process(&adc_results);
        
		int status;
		if(can_handle_interrupt(&candev, &status) == CAN_INTERRUPT) {
			can_msg_t incoming;
			uint8_t status = CAN_OK;
			for (int i=0; i<4 && status == CAN_OK; ++i) {
				status = can_read_message(&candev, &incoming);
				//if (status == CAN_OK)
					// do something with any incoming msgs
			}
        }

        // 200 hz timer
        if (g_timer200_set)
        {
            g_timer200_set = 0;
			//puts_P(PSTR("200hz"));

            // increment the timeouts
            state.timeout++;
            state.comm_timeout++;

            // check for incoming bytes, return positive if bytes received
            if (serial_process(&serial_state, &state.flags, &state.cmd))
                state.comm_timeout = 0;

            adc_process(&adc_results);
        
            // send out packets if ready
            serial_send(&state.cmd);
            
			// find the current time in tenth ms
			uint32_t ct = jiffie();
			// calc the elapsed time in tenth ms
			g_cycle_time = timer_elapsed(lt, ct);
			// reset the last time
			lt = ct;
        }

        // 20 hz timer
        if (g_timer20_set)
        {
            puts_P(PSTR("20hz"));
            
            g_timer20_set = 0;
        }

    }
    return 0;
}
