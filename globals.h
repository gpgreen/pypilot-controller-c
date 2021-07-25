#ifndef GLOBALS_H_
#define GLOBALS_H_

/*-----------------------------------------------------------------------*/

#include <inttypes.h>
#include "can.h"

/*-----------------------------------------------------------------------*/
// the can bus state
extern canbus_state_t g_state;

/*-----------------------------------------------------------------------*/
// the cycle time (approx 80hz) in tenth milliseconds
extern uint32_t g_cycle_time;

/*-----------------------------------------------------------------------*/
// error number
extern volatile uint8_t errcode;

/*-----------------------------------------------------------------------*/
// firmware failed
extern void panic(void);

/*-----------------------------------------------------------------------*/
// bitflags for status
#define SYNC               (0)
#define OVERTEMPFAULT      (1)
#define OVERCURRENTFAULT   (2)
#define ENGAGED            (3)
#define INVALID            (4)
#define PORTPINFAULT       (5)
#define STBDPINFAULT       (6)
#define BADVOLTAGEFAULT    (7)
#define MINRUDDERFAULT     (8)
#define MAXRUDDERFAULT     (9)
#define CURRENTRANGE      (10)
#define BADFUSES          (11)
#define REBOOTED          (12)
#define READQUEUEOVF      (13)
#define WRITEQUEUEOVF     (14)

typedef uint16_t status_flags_t;

/*-----------------------------------------------------------------------*/
enum command_execute {
    Stop = 0,
    Engage,
    Disengage,
    ProcessPacket,
    SendPacket,
    None,
};
enum outgoing_packet {
    Skip,
    Send,
};

typedef struct command_exe 
{
    enum command_execute pending_cmd;
    enum outgoing_packet send_cmd;
    uint8_t in_packet[3];
    uint8_t out_packet[3];
} command_to_execute_t;

/*-----------------------------------------------------------------------*/
#endif  // GLOBALS_H_
