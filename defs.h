/*
 * Define registers and pins
 * atmega328p
 * pypilot-controller board
 */
#ifndef DEFS_H_
#define DEFS_H_

/*-----------------------------------------------------------------------*/
/*
 * Hardware and software revisions
 */
#define HARDWARE_REVISION               2
#define APP_VERSION_MAJOR               0
#define APP_VERSION_MINOR               1

/*-----------------------------------------------------------------------*/
/* functions available */

/* define the size of the fifo buffers for the uart */
#define TX_FIFO_BUFFER_SIZE             128
#define RX_FIFO_BUFFER_SIZE             64

/* define baud rate for serial comm */
#define BAUD                            230400

/* define using Timer2 for jiffies */
#define USE_TIMER2_JIFFIES             1

/*-----------------------------------------------------------------------*/

/* DEBUGGING */
       
//#define MCPDEBUG                    1
//#define CANDEBUG                        1

/*-----------------------------------------------------------------------*/

/* LEDS */

/*-----------------------------------------------------------------------*/
#define MCP2515FILTER                   1
/* MCP clock */
#define MCP2515_16MHZ                   1

/* FIFO lengths */
#define MCP_RECVBUFLEN                  10
#define MCP_ERRBUFLEN                   20

/* MCP interrupt vector on arduino d9 (PB1) (pcint1) */
#define MCP2515_INT_VECT_ANY_CHANGE     1
#define MCP2515_INT_VECT                PCINT0_vect

/* CS pin tied to arduino d10 (PB2)*/
#define DDR_CANCS                       DDRB
#define PORT_CANCS                      PORTB
#define P_CANCS                         2

/* SPI pins tied to arduino d11(MOSI), d12(MISO), d13(SCK)*/
#define DDR_SPI                         DDRB
#define PORT_SPI                        PORTB
#define P_MOSI                          3
#define P_MISO                          4
#define P_SCK                           5

/*-----------------------------------------------------------------------*/
/* Motor pins */

#define DDR_INA                         DDRD
#define DDR_INB                         DDRD
#define PORT_INA                        PORTD
#define PORT_INB                        PORTD
#define P_INA                           2
#define P_INB                           3

/*-----------------------------------------------------------------------*/
#endif  // DEFS_H_
