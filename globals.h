#ifndef GLOBALS_H_
#define GLOBALS_H_

/*-----------------------------------------------------------------------*/

#include <stdint.h>
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

extern const uint8_t LOWCURRENT;

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
// values for packet type fields
// input codes
#define ReprogramCode               (0x19)
#define ResetCode                   (0xe7)
#define CommandCode                 (0xc7)
#define MaxCurrentCode              (0x1e)
#define MaxControllerTempCode       (0xa4)
#define MaxMotorTempCode            (0x5a)
#define RudderRangeCode             (0xb6)
#define RudderMinCode               (0x2b)
#define RudderMaxCode               (0x4d)
#define DisengageCode               (0x68)
#define MaxSlewCode                 (0x71)
#define EEPROMReadCode              (0x91)
#define EEPROMWriteCode             (0x53)
// output codes
#define CurrentCode                 (0x1c)
#define VoltageCode                 (0xb3)
#define ControllerTempCode          (0xf9)
#define MotorTempCode               (0x48)
#define RudderSenseCode             (0xa7)
#define FlagsCode                   (0x8f)
#define EEPROMValueCode             (0x9a)
#define InvalidCode                 (0x00)
// for debugging
#define CurrentFSMCode              (0x20)
#define PreviousFSMCode             (0x21)
#define MsgCode                     (0x22)

typedef uint8_t packet_type_t;

/*-----------------------------------------------------------------------*/

typedef enum command_execute {
    Stop = 0,
    Engage,
    Disengage,
    ProcessPacket,
    SendPacket,
    None,
} command_execute_t;

typedef enum outgoing_packet_cmd {
    Skip,
    Send,
} outgoing_packet_cmd_t;

typedef struct command_exe 
{
    command_execute_t pending_cmd;
    outgoing_packet_cmd_t send_cmd;
    uint8_t in_packet[3];
    uint8_t out_packet[3];
} command_to_execute_t;

/*-----------------------------------------------------------------------*/

typedef enum fsm_states {
    Start,
    WaitEntry,
    Wait,
    Engaged,
    Operational,
    DisengagedEntry,
    Disengaged,
    DetachEntry,
    Detach,
    PowerDown,
} fsm_states_t;

/*-----------------------------------------------------------------------*/

typedef struct fsm_state_memo 
{
    fsm_states_t current;
    fsm_states_t previous;
} fsm_state_memo_t;

/*-----------------------------------------------------------------------*/
// main state for the firmware
typedef struct state 
{
    fsm_state_memo_t machine_state;
    fsm_state_memo_t prev_state;
    uint16_t timeout;
    uint16_t comm_timeout;
    uint16_t command_value;
    uint16_t lastpos;
    uint8_t max_slew_speed;
    uint8_t max_slew_slow;
    uint16_t max_voltage;
    uint16_t max_current;
    uint16_t max_controller_temp;
    uint16_t max_motor_temp;
    uint16_t rudder_min;
    uint16_t rudder_max;
    status_flags_t flags;
    command_to_execute_t cmd;
} state_t;

/*-----------------------------------------------------------------------*/
#endif  // GLOBALS_H_
