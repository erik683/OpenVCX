/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * device_vcx.c - device.h transport backend for the VCX Nano (Phase 2).
 *
 * Speaks the VCX Nano wire protocol (confirmed on a live CAN bus 2026-08-28)
 * ; see notes/message_path_protocol.md):
 *
 *   frame  : BB <escaped(content)> BB
 *   content: 80 <cmd_hi> <cmd_lo> <chan> [payload...] <check>
 *   check  : sum(all content bytes) & 0xFF
 *   escape : any content byte in {BB,EE,DD} -> DD (~byte)
 *   cmd_hi : 0x00 = control command;  0x01 = message TX.  Received messages arrive
 *            under either value, so the receive side keys on cmd_lo alone.
 *
 * Control commands (cmd_hi=0): 8C GetInfo, 40 open, 45 params, 47 filter-init,
 * 42 start, 48 add-filter, 43 stop, 41 close. Each gets a reply < 80 00 <op> ..|status.
 * Message TX (cmd_hi=1, cmd_lo=0) gets no reply and no transmit confirmation: no vendor
 * capture contains a TxDone frame, so the DLL synthesizes one (see vcx_send_msg).
 * RX messages (cmd_lo=0): < 80 <00|01> 00 <chan> | <rxstatus u32><len u16><canid u32><data>.
 * Bench captures show unsolicited bus traffic on cmd_hi=0; a 2026-08-28 car capture
 * delivered an ISO15765 ECU reply on cmd_hi=1.
 *
 * SAFETY: never emits device reset 0x8D, bootloader 0xF0, serial rewrite 0xF7, or config-flash 0xA3.
 * The transient PASSTHRU session is established directly with A0/84/A1/A2; no VX software is
 * loaded or invoked. OPEN status 0xFE == session absent; 0x9A == engine unavailable.
 */
#include "device.h"
#include "pt_wire.h"
#include "j2534_defs.h"
#include "pt_validate.h"
#include "vcx_transport.h"

#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#define VCX_OP_GETINFO 0x8C
#define VCX_OP_OPEN    0x40
#define VCX_OP_CLOSE   0x41
#define VCX_OP_START   0x42
#define VCX_OP_STOP    0x43
#define VCX_OP_PARAMS  0x45
#define VCX_OP_FILTINIT 0x47
#define VCX_OP_ADDFILT 0x48
#define VCX_OP_PERIODIC_CTRL 0x49   /* value 01 = enable service, 02 = clear all */
#define VCX_OP_PERIODIC_ADD  0x4A   /* add one timer record, or stop one (zero record) */
#define VCX_OP_MSG     0x00      /* cmd_lo for TX (cmd_hi=1) and RX (cmd_hi=0) */
#define VCX_OP_LICENSE_QUERY 0x84
#define VCX_OP_LICENSE_DH    0xA0
#define VCX_OP_LICENSE_SET   0xA1
#define VCX_OP_LICENSE_GET   0xA2
#define VCX_STATUS_OK  0x00
#define VCX_XXTEA_DELTA 0x9E3779B9u
#define VCX_OUTER_MARKER 0xA567A567u
#define VCX_PASSTHRU_ID 0xDFB986A5u
#define VCX_LICENSE_REFRESH_MS 240000u
#define VCX_MAX_FILTERS  16
#define VCX_FILTER_MAXLEN 32
/* Command 0x48 record: [count] then per filter [slot type flag size] + arrays.
 * The slot byte is the firmware slot index (pfilter_list_add reduces it %64)
 * and must be distinct per filter: a record sent to an occupied slot replaces
 * that filter.  The DLL uses the record's host index as its slot. */
#define VCX_FILT_SLOT_BYTE 1
#define VCX_RESP_QUEUE_CAP 4

/* Device comm-param ids for opcode 0x45, read out of the firmware's per-engine
 * set_comparam switches (ISO15765 0x08018B00, raw CAN 0x0803740A). */
#define VCX_PID_BUS_PIN        0x0002  /* value = pin1<<24 | pin2<<16 */
#define VCX_PID_BAUDRATE       0x0003
#define VCX_PID_CAN_BUS        0x0100  /* which CAN controller: 0 = CAN1, 1 = CAN2 */
#define VCX_PID_ISO15765_BS    0x8005
#define VCX_PID_ISO15765_STMIN 0x8006
/* ISO9141/ISO14230 fast-init wake/idle timing, read out of the UART setter
 * switches (ISO9141 0x08032A1E, ISO14230 0x080310DA).  0x08032858's fast-init
 * loop waits on these fields divided by 100 against a free-running counter,
 * the same scale the vendor firmware uses for periodic-message intervals
 * (notes/hds_required_protocol_trace.md: "interval is microseconds, while
 * the J2534 API supplies milliseconds") -- so these need the same ms->us
 * conversion, done in j2534_to_vcx_value() below. */
#define VCX_PID_ISO_P1_MAX     0x0041
#define VCX_PID_ISO_P2_MAX     0x0042
#define VCX_PID_ISO_P3_MIN     0x0043
#define VCX_PID_ISO_P4_MIN     0x0044
#define VCX_PID_ISO_TIDLE      0x0045
#define VCX_PID_ISO_TWUP       0x0046
#define VCX_PID_ISO_TINIL      0x0047
/* Five-baud (slow) init windows, microseconds. Names are the ISO9141 setter's
 * own log strings (PID_SLOW_W1MAX..W4MIN); ISO9141, ISO14230, KW82 and KW1281
 * all accept them and feed the shared driver uart_slow_init_start
 * (notes/uart_slow_init_firmware.md). PID_SLOW_MODE (0x48) is overwritten by
 * the firmware's key-byte rule and PID_SLOW_W4MAX (0x4D) has no J2534
 * counterpart, so neither is sent. */
#define VCX_PID_SLOW_W1_MAX    0x0049
#define VCX_PID_SLOW_W2_MAX    0x004A
#define VCX_PID_SLOW_W3_MAX    0x004B
#define VCX_PID_SLOW_W4_MIN    0x004C
#define VCX_PID_UART_FORMAT     0x0400  /* data bits | parity<<8 | stop bits<<16 */
/* J1850 PWM/VPW functional (all-call) address, read out of the J1850 setter
 * switches (PWM 0x0803690C, VPW 0x08036E7A) alongside node address 0x8501. */
#define VCX_PID_J1850_NODE_ADDR  0x8501
#define VCX_PID_J1850_FUNCT_ADDR 0x8502
/* UART/K-line checksum enable, read out of the UART setter switch case 0x21
 * (analysis/recovered/vcx_uart_protocol_recovered.h): 1 = engine appends and
 * verifies the ISO checksum, 0 = the application owns it (the J2534
 * ISO9141_NO_CHECKSUM connect flag).  Distinct from the ISO timing PIDs above
 * only in meaning; same 0x45 push. */
#define VCX_PID_UART_CHECKSUM    0x0021
static uint16_t proto_to_engine(uint32_t proto)
{
    switch (proto) {
    case J2534_J1850VPW:  return 0x8505;
    case J2534_J1850PWM:  return 0x8606;
    case J2534_ISO9141:   return 0x9104;
    case J2534_ISO14230:  return 0x9004;
    case J2534_CAN:       return 0x8101;
    case J2534_ISO15765:  return 0x8001;
    /* J2534-2 pin-select variants. Same engines as their base protocols --
     * the pins are chosen with J1962_PINS, not by the engine id. Mapping
     * transcribed from the vendor DLL's own mapper (notes/host_protocol_map.md);
     * without these the DLL advertised ISO15765_PS in the registry and then
     * rejected it with ERR_INVALID_PROTOCOL_ID, which is what stopped FORScan
     * from ever reaching MS-CAN. */
    case J2534_2_J1850VPW_PS:       return 0x8505;
    case J2534_2_J1850PWM_PS:       return 0x8606;
    case J2534_2_ISO9141_PS:        return 0x9104;
    case J2534_2_ISO14230_PS:       return 0x9004;
    case J2534_2_CAN_PS:            return 0x8101;
    case J2534_2_ISO15765_PS:       return 0x8001;
    case J2534_2_SW_ISO15765_PS:    return 0x8001;
    case J2534_2_SW_CAN_PS:         return 0x8101;
    case J2534_2_UART_ECHO_BYTE_PS: return 0x9204;
    case J2534_2_HONDA_DIAGH_PS:    return 0x9104;
    case J2534_2_TP2_0_PS:          return 0x8401;
    case J2534_2_FT_CAN_PS:         return 0x8101;
    case J2534_2_FT_ISO15765_PS:    return 0x8001;
    case VCXID_J1939_PS:  return 0x8201;
    case VCXID_J1708_PS:  return 0x8707;
    case VCXID_LIN:       return 0x9D04;
    case VCXID_TP20:      return 0x8401;
    case VCXID_TP16:      return 0x8301;
    case VCXID_KW82:      return 0x9B04;
    default:              return 0;
    }
}

/* CAN-family engines share the 4-byte-id filter payload and the ISO15765
 * transmit semantics; keying off the engine keeps every _PS alias in step. */
static bool engine_is_can(uint16_t engine)
{
    return engine == 0x8001 || engine == 0x8101 || engine == 0x8201 ||
           engine == 0x8401 || engine == 0x8301 || engine == 0x8002 || engine == 0x8102;
}

static bool engine_is_iso15765(uint16_t engine)
{
    return engine == 0x8001 || engine == 0x8002;
}

/* UART/K-line engines: the firmware drives all of these through uart_hw_set_pin
 * (0x0801C890), whose only wired route is pin 7 via kline_claim_pin_7.  Engine
 * ids in the 0x9??4 family -- ISO9141/HONDA_DIAGH 0x9104, ISO14230 0x9004,
 * UART_ECHO_BYTE 0x9204, LIN 0x9D04. */
static bool engine_is_kline(uint16_t engine)
{
    return engine == 0x9004 || engine == 0x9104 ||
           engine == 0x9204 || engine == 0x9B04 || engine == 0x9D04;
}

/* The K-line engines whose write path runs the shared five-baud driver and
 * whose setters take PIDs 0x48..0x4D; LIN has neither. */
static bool engine_has_slow_init(uint16_t engine)
{
    return engine == 0x9004 || engine == 0x9104 || engine == 0x9204 || engine == 0x9B04;
}

/* ---- state ---- */
static HINSTANCE   s_module;
static HANDLE      s_com = INVALID_HANDLE_VALUE;
static CRITICAL_SECTION s_io;      /* serializes host writes + control transactions */
static CRITICAL_SECTION s_chan;    /* guards the channel table + rx queues */
static CRITICAL_SECTION s_resp_cs; /* guards the control-response queue */
static CRITICAL_SECTION s_license; /* serializes the whole license_unlock() handshake */
static bool        s_cs_ready;
/* Serializes the physical link's lifecycle -- connect, disconnect and the
 * cross-process handoff below -- because the handoff watcher can drop the port
 * while an application thread is trying to take it.  Outermost lock: it is
 * taken before s_io/s_chan/s_license and never by the reader or periodic
 * threads. */
static CRITICAL_SECTION s_link;
static bool        s_session_active;  /* between PassThruOpen and PassThruClose */
static char        s_port_name[16];   /* the port this process currently holds */
static HANDLE      s_release_evt;     /* named; another process asks us to let go */
static HANDLE      s_busy_evt;        /* named; we answer "no, I am using it" */
static HANDLE      s_handoff_thread;  /* services that request; lives to dev_shutdown */
static HANDLE      s_handoff_quit;
static bool        s_connect_busy;    /* last dev_connect failed: port held elsewhere */
static char        s_busy_port[16];
static DWORD       s_last_open_err;   /* GetLastError from the last try_open_once */
static bool        s_open_quiet;      /* inside the handoff poll: say it once, not 60x */
static volatile LONG s_link_dead;     /* reader hit an unrecoverable read error */
static dev_channel_t s_channels[DEV_MAX_CHANNELS];
static uint32_t    s_filter_seq[DEV_MAX_CHANNELS];
static HANDLE      s_reader;
static volatile LONG s_reader_run;
/* Queue replies because a second can arrive before the waiter consumes the first,
 * which the old single slot silently overwrote.  With no request sequence number,
 * flush the queue under s_io before each write and match replies by opcode and channel.
 * A late reply arriving after its timeout but after a new same-opcode/channel write can
 * still be misattributed.  We deliberately accept that: logs/openvcx.log records the
 * failure once -- 24 consecutive 3 s 0x86 timeouts, then unaided recovery and 399 more
 * good 0x86 transactions -- and across the whole 83 s outage it sent not one frame: no
 * late reply, and no backlog on the way back.  Here, timeout means a
 * dropped request, not an in-flight reply, so resynchronizing would only waste the
 * live session's remaining transactions. */
static HANDLE      s_resp_event;    /* signaled when the response queue is nonempty */
typedef struct {
    int len;
    uint8_t data[4300];
} control_response_t;
static control_response_t s_resp_queue[VCX_RESP_QUEUE_CAP];
static unsigned    s_resp_head;
static unsigned    s_resp_tail;
static unsigned    s_resp_count;
static char        s_logpath[MAX_PATH];
static volatile LONG s_logging;
static CRITICAL_SECTION s_log_cs;  /* serializes log lines across threads */
static bool        s_log_cs_ready; /* remains valid through DLL shutdown */
static FILE       *s_logf;         /* held open for the process; see dev_log */
static bool        s_log_failed;   /* the path could not be opened; stop retrying */
static int         s_hex_max;      /* bytes of a dump to print; 0 = no limit */
static int         s_rx_log_every = 256; /* after first 64 message frames */
static int         s_log_level = 2;
static bool        s_log_header_done;

/* Everything the transport discards or fails on.  A frame this DLL cannot use
 * is still evidence -- of a firmware quirk, a cable fault, or a bug here -- so
 * each one is logged when it happens and counted for the teardown summary. */
static struct {
    volatile LONG rx_bytes;         /* bytes read off the port */
    volatile LONG rx_frames;        /* well-formed frames handed to handle_frame */
    volatile LONG rx_messages;      /* well-formed vehicle-message frames */
    volatile LONG rx_noise_bytes;   /* bytes seen before the first delimiter */
    volatile LONG rx_bad_csum;      /* frame checksum mismatch */
    volatile LONG rx_runt;          /* frame too short to carry a header */
    volatile LONG rx_bad_magic;     /* content[0] != 0x80 */
    volatile LONG rx_oversize;      /* frame longer than the reassembly buffer */
    volatile LONG rx_short_msg;     /* message frame shorter than its 10-byte header */
    volatile LONG rx_len_clamped;   /* declared payload length exceeded the frame */
    volatile LONG rx_unknown_chan;  /* message for a channel we have not opened */
    volatile LONG rx_queue_full;    /* per-channel receive queue overflowed */
    volatile LONG rx_queue_alloc_failed; /* message dropped because malloc failed */
    volatile LONG resp_dropped;     /* control reply evicted from a full queue */
    volatile LONG resp_stale;       /* reply discarded: wrong opcode/channel */
    volatile LONG resp_unusable;    /* reply discarded: runt or bad magic */
    volatile LONG tx_unknown_chan;  /* tx confirmations with no open channel */
    volatile LONG xact_timeout;     /* control transaction gave up */
    volatile LONG read_errors;      /* ReadFile failed */
    volatile LONG write_errors;     /* WriteFile failed or wrote short */
    volatile LONG comm_errors;      /* ClearCommError flags (overrun/framing/parity) */
    /* Host repeat-message scheduler (HDS 0x8004): K-line turn-taking. */
    volatile LONG repeat_no_reply;  /* resent after repeat_reply_timeout_ms of silence */
    volatile LONG repeat_malformed; /* reply ignored by the completion test */
    /* PASSTHRU licensing/session cost -- how much the entitlement machinery
     * actually costs live operation (issue #5 criterion D). */
    volatile LONG lic_unlock_attempts;  /* license_unlock() entered */
    volatile LONG lic_unlock_ok;        /* session object installed */
    volatile LONG lic_unlock_fail;      /* handshake did not complete */
    volatile LONG lic_unlock_ms_total;  /* summed wall-time in the 5-xact handshake */
    volatile LONG lic_unlock_ms_max;    /* worst single handshake */
    volatile LONG lic_refresh_run;      /* pre-connect relicense actually executed */
    volatile LONG lic_refresh_skipped;  /* pre-connect: session still valid, no relicense */
    volatile LONG lic_open_gate_fe;     /* OPEN returned 0xFE (session-absent gate) */
    volatile LONG lic_open_recovered;   /* 0xFE OPEN healed by relicense + retry */
} s_stat;

#define STAT_BUMP(field) InterlockedIncrement(&s_stat.field)

/* Log the first 64 occurrences of an event, then every 256th.  A wire gone to
 * noise must not push the useful history out of the file, but the count keeps
 * climbing and the summary reports the true total, so nothing goes unseen. */
static bool stat_verbose(LONG n)
{
    return n <= 64 || (n % 256) == 0;
}

static bool rx_log_sample(LONG n)
{
    return s_rx_log_every <= 1 || n <= 64 || (n % s_rx_log_every) == 0;
}

/* Bump field and, if the running count passes stat_verbose's sampling gate,
 * log a line with that count as the final "(#%ld)" argument. Covers the
 * common "bump, sample-gate, log one line" shape used throughout the
 * reader/transaction paths below; sites with extra work inside the gate, or
 * that always log, stay spelled out. */
#define STAT_LOG(field, ...) \
    do { \
        LONG _stat_n = STAT_BUMP(field); \
        if (stat_verbose(_stat_n)) dev_log(__VA_ARGS__, _stat_n); \
    } while (0)
#define STAT_LOG_HEX(field, data, len, ...) \
    do { \
        LONG _stat_n = STAT_BUMP(field); \
        if (stat_verbose(_stat_n)) dev_log_hex((data), (len), __VA_ARGS__, _stat_n); \
    } while (0)

static uint8_t     s_license_key_prefix[8];
static bool        s_have_license_key_prefix;
/* The 64-byte VCX_CmdDevGetInfo block from the identifying probe, served
 * verbatim as PT_CMD_VERSION so firmware identity comes from the device. */
static uint8_t     s_devinfo[64];
static bool        s_have_devinfo;
static DWORD       s_license_tick;
/* Age (ms) past which do_connect pre-emptively reinstalls the session before
 * opening a channel.  The firmware's OPEN gate (0x0801AF28) has no session-age
 * timer -- the session object at SRAM 0x2000D03C persists until a device reset
 * -- so this pre-emptive refresh is belt-and-suspenders over OPEN's own
 * reactive 0xFE recovery.  Default 240000 preserves historical behaviour;
 * license_refresh_ms=0 in vcx_nano.ini selects reactive-only (install once,
 * then rely on the 0xFE retry), which removes the periodic 5-transaction
 * relicense from the connect critical path.  Tune against the lic_* counters
 * in dev_log_stats. */
static DWORD       s_license_refresh_ms = VCX_LICENSE_REFRESH_MS;
/* ===================================================================
 * Programming-voltage capability -- HARDWARE-SPECIFIC, DO NOT GENERALISE
 * ===================================================================
 * Everything below is calibrated to the bench-measured behaviour of the
 * original reverse-engineering development unit: a Ford/Mazda branded VCX Nano.
 * On it, the entire programmable-voltage feature reduces to two facts:
 *
 *   - The ONLY pin that can source a programming voltage is J1962 pin 13
 *     (Ford FEPS).  It is a hard on/off switch, not a DAC: any non-zero
 *     request comes out as ~18 V, and 0/OFF turns it off.  Pins
 *     6/9/11/12/14/15 source nothing.
 *   - The ONLY pin that can be *sensed* is pin 16 (VBATT), via READ_VBATT.
 *     Pin 13 has no readback path, so READ_PROG_VOLTAGE has no honest
 *     numeric answer on this unit.
 *
 * Firmware analysis (notes/firmware_power_architecture.md) since established
 * WHY, and every rule below now rests on a mechanism rather than on the bench
 * alone.  The J1962 switch fabric is five I2C I/O expanders behind a RAM
 * shadow, and the function that flushes that shadow to them (bus_mux_out) is
 * `bx lr` in 1.9.4.2; the rail's level control (bsp_vddp_pwm_set) logs its
 * computed duty cycle and returns.  The only two paths that reach hardware are
 * the ones that bypass the fabric -- the rail enable GPIO and the dedicated
 * VBATT divider into the ADC -- which is exactly the pair the bench found.
 * A sibling image in the lineup still carries both bodies intact, so these are
 * removals from this build, not absent hardware.
 *
 * !!! Other units in the Nano lineup are built for other OEMs and WILL
 * !!! differ.  A GM/VAG/PSA/etc. variant may source on a different pin,
 * !!! support a real variable rail, expose more than one sense channel, or
 * !!! map FEPS somewhere other than pin 13.  Do not copy VCX_FEPS_PIN, the
 * !!! "18 V on/off" collapse, or the "pin 16 only" sense assumption into a
 * !!! build for another manufacturer without re-measuring that board first.
 * !!! Treat these constants as the Ford calibration, not a device fact. */

/* How much of the (limited) truth the DLL exposes.  Tunable because some
 * tools (e.g. HDS) abort a session if a boilerplate voltage call is refused,
 * so a bench operator can trade J2534 correctness for app compatibility.
 * Ordered by increasing permissiveness; see do_set_prog_voltage /
 * READ_PROG_VOLTAGE for exactly what each does.  Select with voltage_policy=
 * in vcx_nano.ini or the VCX_NANO_VOLTAGE_POLICY env var.  Default: strict. */
typedef enum {
    PROG_V_STRICT = 0, /* J2534 spec: refuse anything the hardware can't do */
    PROG_V_HONEST = 1, /* same refusals, plus the answerable no-lie extras   */
    PROG_V_LOOSE  = 2, /* never block a set; reads stay truthful (~0)        */
    PROG_V_COMPAT = 3, /* stock illusion: always OK, echo target on read     */
} prog_v_policy_t;
/* strict and honest are the two truthful tiers: neither ever reports success
 * for something the silicon did not do.  They differ only in how much of what
 * IS knowable they are willing to answer -- see do_set_prog_voltage and
 * READ_PROG_VOLTAGE. */
#define PROG_V_TRUTHFUL(pol) ((pol) == PROG_V_STRICT || (pol) == PROG_V_HONEST)
static prog_v_policy_t s_prog_policy = PROG_V_STRICT;

/* J2534 validation strictness; see dev_strict_validation() in device.h. */
static bool s_strict_validation = false;
bool dev_strict_validation(void) { return s_strict_validation; }

#define VCX_FEPS_PIN        13u          /* Ford calibration -- see warning above */
#define VCX_VBATT_PIN       16u          /* the one pin with a sense path         */
#define J2534_VOLTAGE_OFF   0xFFFFFFFFu  /* J2534 SetProgrammingVoltage sentinel  */
#define J2534_SHORT_TO_GND  0xFFFFFFFEu  /* J2534 sentinel (no rail here to short)*/

/* The FEPS rail is a fixed ~18 V high-side switch, not a DAC.  A millivolt
 * request is answerable only if it lands in this band; any other level is
 * something the rail cannot produce, and saying otherwise would deliver 18 V to
 * a caller that asked for 5 V. */
#define VCX_FEPS_NOMINAL_MV 18000u
#define VCX_FEPS_MIN_MV     16200u       /* nominal -10% */
#define VCX_FEPS_MAX_MV     19800u       /* nominal +10% */

/* What a J1962 pin can physically do on this board (see the capability warning
 * block above -- this is the Ford calibration, not a device fact). */
typedef enum {
    PIN_CAP_NONE = 0,   /* neither driven nor measured: 6, 9, 11, 12, 14, 15 */
    PIN_CAP_SENSE,      /* readable only: 16 (VBATT)                        */
    PIN_CAP_SOURCE,     /* can be energised: 13 (FEPS)                      */
} pin_cap_t;

static pin_cap_t pin_capability(uint32_t pin)
{
    if (pin == VCX_FEPS_PIN)  return PIN_CAP_SOURCE;
    if (pin == VCX_VBATT_PIN) return PIN_CAP_SENSE;
    return PIN_CAP_NONE;
}

/* Can PassThruSetProgrammingVoltage do anything at all to this pin? */
static bool pin_can_source(uint32_t pin)
{
    return pin_capability(pin) == PIN_CAP_SOURCE;
}

/* Is `pin` a legal PinNumber argument to PassThruSetProgrammingVoltage at all?
 * This is a question about the API, not about this board: J2534-1 enumerates
 * the J1962 pins programming voltage may be requested on, plus 0 for "release
 * whatever is energised".  Pin 16 is deliberately absent -- it is battery, an
 * input, never a programmable output.
 *
 * The distinction matters for which error we owe the caller.  ERR_PIN_INVALID
 * says the argument itself is wrong (not a pin, or in use), and invites a tool
 * to release something and retry.  ERR_NOT_SUPPORTED says the request was
 * well-formed and this device cannot honour it -- which is the true statement
 * about pin 9 on a board where only pin 13 is wired, and it is terminal, so a
 * capability probe stops instead of circling. */
static bool pin_is_prog_voltage_arg(uint32_t pin)
{
    switch (pin) {
    case 0: case 6: case 9: case 11: case 12: case 13: case 14: case 15:
        return true;
    default:
        return false;
    }
}

/* Pin last energised by PassThruSetProgrammingVoltage (0 = off / none), and the
 * target voltage the app asked for, so compat-mode READ_PROG_VOLTAGE can echo a
 * plausible reading back.  On this unit only VCX_FEPS_PIN ever lands here. */
static uint8_t     s_prog_pin;
static uint32_t    s_prog_target_mv;
/* Set before an ON write: even a lost reply may leave FEPS energised. */
static bool        s_prog_maybe_on;

/* Cached device ADD-FILTER records let us remove one filter by rebuilding the
 * device bank, which has no per-filter-remove command. */
static uint8_t  s_filt[DEV_MAX_CHANNELS][VCX_MAX_FILTERS][VCX_FILTER_MAXLEN];
static int      s_filt_len[DEV_MAX_CHANNELS][VCX_MAX_FILTERS];
static uint32_t s_filt_id[DEV_MAX_CHANNELS][VCX_MAX_FILTERS];
static int      s_filt_n[DEV_MAX_CHANNELS];

/* BLOCK_FILTER is matched host-side.  The firmware command-0x48 record for a
 * block was never captured, so rather than guess a wire format the DLL drops
 * matching frames as the reader enqueues them (route_rx).  A rule is a
 * mask/pattern pair over the leading bytes of a received message; ids share the
 * s_filter_seq space with pass/flow filters so StopMsgFilter and CLEAR reach
 * both.  Guarded by s_chan (read on the reader thread, written on the app
 * thread; a block add never touches the wire, so no s_io ordering concern). */
#define VCX_MAX_BLOCK 8
typedef struct {
    bool in_use; uint32_t id; uint8_t len; uint8_t mask[12], patt[12];
} block_rule_t;
static block_rule_t s_block[DEV_MAX_CHANNELS][VCX_MAX_BLOCK];

static bool rx_blocked_locked(uint8_t chan, const uint8_t *data, uint16_t len)
{
    for (int i = 0; i < VCX_MAX_BLOCK; i++) {
        block_rule_t *b = &s_block[chan][i];
        if (!b->in_use || b->len == 0 || len < b->len) continue;
        bool hit = true;
        for (int k = 0; k < b->len; k++)
            if ((data[k] & b->mask[k]) != (b->patt[k] & b->mask[k])) { hit = false; break; }
        if (hit) return true;
    }
    return false;
}

#define DEV_MAX_PERIODIC 8
/* The firmware's per-engine VcxPeriodicMessageService holds ten timer records
 * (notes/tp_engine_vtable_recovery.md / periodic_timer_vendor_capture). */
#define VCX_FW_PERIODIC_SLOTS 10
typedef struct {
    bool in_use;
    uint8_t chan;
    uint32_t id, interval;   /* interval in J2534 ms */
    DWORD next;
    uint32_t txflags;
    uint16_t len;
    uint8_t data[12];
    bool fw;                 /* true: firmware timer slot; false: host scheduler */
    uint8_t fw_slot;         /* device timer slot when fw */
} periodic_t;
static CRITICAL_SECTION s_per;
static periodic_t s_periodic[DEV_MAX_PERIODIC];
/* Bitmap of device timer slots currently in use per channel (fw path). */
static uint16_t s_fw_slot_used[DEV_MAX_CHANNELS];
static uint32_t s_periodic_seq;
static HANDLE s_periodic_thread;
static volatile LONG s_periodic_run;

static long periodic_stop_all(void);
static long periodic_clear_chan(uint8_t chan);

/* One repeat per channel: the protocol has no transaction IDs with which to
 * associate concurrent repeat replies. Retain stopped records until STOP.
 *
 * K-line is half-duplex, so a repeat is a request/response exchange, not a
 * timer: after each send the scheduler waits for the ECU's reply (or
 * s_repeat_reply_timeout_ms of silence) and then P3_MIN before the next one.
 * B001 sent on a bare 30 ms timer while Honda's ABS took ~77 ms to answer;
 * tester and ECU talked over each other, and the garbled replies falsely
 * completed the repeat (live session 2026-09-28, case C5). */
typedef struct {
    bool used, active, sent;
    bool awaiting;           /* sent, and the ECU has not yet taken its turn */
    bool honda_framed;       /* request is [hdr][len][data][cs]: replies must be too */
    bool no_reply;           /* reply window lapsed; the next send is a resend */
    uint32_t id, interval, condition, flags;
    uint32_t p3_ms;          /* channel P3_MIN at START, in the DLL's ms units */
    uint64_t generation;
    DWORD next;              /* earliest next send once the ECU has answered */
    DWORD sent_at;
    DWORD quiet_until;       /* last ECU frame + P3_MIN: no tester send before */
    long error;
    uint16_t tx_len, filter_len;
    uint8_t tx[4128], mask[4128], pattern[4128];
} repeat_t;
static repeat_t s_repeat[DEV_MAX_CHANNELS]; /* s_chan; send barrier is s_io */
static uint32_t s_repeat_seq;
/* Mirrors s_repeat[ch].awaiting for the periodic scheduler, which runs on the
 * same worker thread as repeat_tick and must not open its own exchange while a
 * repeat reply is outstanding. */
static volatile LONG s_repeat_awaiting[DEV_MAX_CHANNELS];
#define REPEAT_REPLY_TIMEOUT_MS 300
static DWORD s_repeat_reply_timeout_ms = REPEAT_REPLY_TIMEOUT_MS;
/* vcx_nano.ini fast_init_timeout_ms: a fixed FAST_INIT wait; 0 derives it from
 * the channel's timing (dev_fast_init_window). */
static DWORD s_fast_init_timeout_ms;
static DWORD WINAPI periodic_proc(LPVOID arg);

#ifdef VCX_REPEAT_TEST
static DWORD repeat_clock(void) { return repeat_test_now ? repeat_test_now : GetTickCount(); }
#else
static DWORD repeat_clock(void) { return GetTickCount(); }
#endif

/* Honda K-line frame: [header][total length][data...][two's-complement sum].
 * The ABS clear's genuine reply 01 04 31 CA passes; the collision debris seen
 * live (04 31 CA, 00 31 CA, 31 CA, F0 11 EA) fails on length or checksum. */
static bool honda_frame_valid(const uint8_t *data, uint16_t len)
{
    if (len < 3 || data[1] != len) return false;
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; ++i) sum = (uint8_t)(sum + data[i]);
    return sum == 0;
}

static void repeat_set_awaiting_locked(uint8_t chan, bool awaiting)
{
    s_repeat[chan].awaiting = awaiting;
    InterlockedExchange(&s_repeat_awaiting[chan], awaiting ? 1 : 0);
}

/* Every non-echo frame on the channel ends the ECU's turn on the wire, whether
 * or not it is usable, blocked by a filter, or clamped -- so the gate opens for
 * all of them.  Only a whole, unblocked frame (whole) may complete the repeat. */
static void repeat_rx_locked(dev_channel_t *c, uint8_t chan, uint32_t status,
                             const uint8_t *data, uint16_t len, bool whole)
{
    repeat_t *r = &s_repeat[chan];
    if (!r->used || !r->active || r->generation != c->generation ||
        (status & J2534_RX_TX_MSG_TYPE)) return;
    DWORD now = repeat_clock();
    if (r->awaiting) repeat_set_awaiting_locked(chan, false);
    r->quiet_until = now + r->p3_ms;
    if ((int32_t)(now + r->p3_ms - r->next) > 0) r->next = now + r->p3_ms;
    if (!whole || !r->sent || status != 0 || len < r->filter_len) return;
    if (r->honda_framed && !honda_frame_valid(data, len)) {
        LONG n = STAT_BUMP(repeat_malformed);
        if (stat_verbose(n))
            dev_log_hex(data, len, "repeat id=%lu ch=%u ignored malformed reply "
                        "(length/checksum) len=%u (#%ld)", (unsigned long)r->id,
                        chan + 1, len, n);
        return;
    }
    bool match = true;
    for (unsigned i = 0; i < r->filter_len; ++i)
        if ((data[i] & r->mask[i]) != r->pattern[i]) { match = false; break; }
    if ((r->condition == 0 && match) || (r->condition == 1 && !match)) {
        r->active = false;
        dev_log("repeat id=%lu ch=%u completed on RX match=%d", (unsigned long)r->id, chan + 1, match);
    }
}
static long release_prog_voltage(void);
static bool keep_warm_enabled(void);

/* Cross-process port handoff.  Windows serial handles are opened non-shared, so
 * a warm link held across a logical close locks every other process out of the
 * device.  HDS does exactly that: it closes the port in testman.exe and, ~300 ms
 * later, launches DataListClient.exe or DTCMonitor.exe expecting to find the
 * interface free (see notes/hds_multiprocess_handoff.md).  A holder therefore
 * publishes a named event while it owns a port, and gives the port up on request
 * whenever it has no J2534 session of its own open.
 *
 * WAIT bounds how long a newcomer waits for the holder to let go; the holder
 * looks for a request every WATCH ms, so the observed handoff costs about
 * WATCH + the teardown rather than the full COM1-32 scan. */
/* The ceiling is real elapsed time, not poll count: a single try_open_once can
 * itself burn ~800 ms when the port exists but the USB bridge is still
 * rebooting, so counting polls would let the "3000 ms" wait run for tens of
 * seconds.  8 s covers the worst case observed in the field -- holder releases,
 * the CH343 bridge re-enumerates, and the port reappears about 5 s later. */
#define PORT_HANDOFF_WAIT_MS   8000
#define PORT_HANDOFF_POLL_MS     50
#define PORT_HANDOFF_WATCH_MS   100
static void handoff_publish(const char *port);
static void handoff_withdraw(void);

static bool logging_enabled(void)
{
    return InterlockedCompareExchange(&s_logging, 0, 0) != 0;
}

bool dev_log_enabled(void) { return logging_enabled(); }
bool dev_log_verbose(void) { return logging_enabled() && s_log_level >= 2; }

static void log_header_locked(void);

/* Open the log once and keep the handle.  The old open/append/close per line
 * cost two syscalls on the hot receive path -- enough to perturb the very
 * K-line and P2 timings the log exists to measure -- and left line order
 * between the reader and application threads up to the filesystem. */
static FILE *log_file_locked(void)
{
    if (s_logf || s_log_failed) return s_logf;
    s_logf = fopen(s_logpath, "a");
    if (!s_logf) {
        int err = errno;
        char debug[2 * MAX_PATH + 96];
        snprintf(debug, sizeof(debug),
                 "OpenVCX: failed to open diagnostic log '%s' (errno=%d)\n",
                 s_logpath, err);
        OutputDebugStringA(debug);
        s_log_failed = true;
        return NULL;
    }
    /* Buffered writes avoid turning every received frame into a disk sync. */
    setvbuf(s_logf, NULL, _IOFBF, 1 << 16);
    if (!s_log_header_done) { s_log_header_done = true; log_header_locked(); }
    return s_logf;
}

/* Timestamps in the vendor's format, so this log and a VCX.DLL capture of the
 * same session read side by side -- and so "did the response arrive before the
 * read timed out?" is answerable from the log alone.  The thread id is what
 * separates the reader thread's frames from the calling application's, which a
 * single interleaved column cannot otherwise distinguish. */
static void log_stamp_locked(FILE *f)
{
    SYSTEMTIME ts; GetLocalTime(&ts);
    fprintf(f, "[%02u:%02u:%02u.%03u t=%04lX] ", ts.wHour, ts.wMinute,
            ts.wSecond, ts.wMilliseconds, (unsigned long)GetCurrentThreadId());
}

/* Set by every buffered write, cleared by a flush: lets the reader commit a
 * passive RX burst without waiting for the next TX or teardown. */
static volatile LONG s_log_dirty;
static volatile DWORD s_log_flushed_at;

void dev_log(const char *fmt, ...)
{
    if (!logging_enabled()) return;
    EnterCriticalSection(&s_log_cs);
    FILE *f = logging_enabled() ? log_file_locked() : NULL;
    if (f) {
        InterlockedExchange(&s_log_dirty, 1);
        log_stamp_locked(f);
        va_list ap; va_start(ap, fmt); vfprintf(f, fmt, ap); va_end(ap);
        fputc('\n', f);
    }
    LeaveCriticalSection(&s_log_cs);
}

void dev_log_hex(const uint8_t *p, int n, const char *fmt, ...)
{
    if (!logging_enabled()) return;
    if (n < 0) n = 0;
    int shown = (s_hex_max > 0 && n > s_hex_max) ? s_hex_max : n;
    EnterCriticalSection(&s_log_cs);
    FILE *f = logging_enabled() ? log_file_locked() : NULL;
    if (f) {
        InterlockedExchange(&s_log_dirty, 1);
        log_stamp_locked(f);
        va_list ap; va_start(ap, fmt); vfprintf(f, fmt, ap); va_end(ap);
        /* The true length is always in the caller's "len=" field; say plainly
         * when the dump behind it is partial rather than let a short line read
         * as a short frame. */
        if (shown < n) fprintf(f, " shown=%d", shown);
        fputc(':', f);
        static const char hexdig[] = "0123456789ABCDEF";
        /* One buffered fwrite per chunk instead of a formatted call per byte:
         * a 4 KB frame is 12 KB of text on a path the reader thread runs. */
        char chunk[3 * 256];
        for (int i = 0; i < shown; ) {
            int o = 0;
            for (int j = 0; j < 256 && i < shown; j++, i++) {
                chunk[o++] = ' ';
                chunk[o++] = hexdig[p[i] >> 4];
                chunk[o++] = hexdig[p[i] & 0x0F];
            }
            fwrite(chunk, 1, (size_t)o, f);
        }
        fputc('\n', f);
    }
    LeaveCriticalSection(&s_log_cs);
}

void dev_log_flush(void)
{
    if (!logging_enabled()) return;
    EnterCriticalSection(&s_log_cs);
    if (logging_enabled() && s_logf) fflush(s_logf);
    InterlockedExchange(&s_log_dirty, 0);
    s_log_flushed_at = GetTickCount();
    LeaveCriticalSection(&s_log_cs);
}

/* Commit output that has sat in the buffer for max_age_ms.  Called from the
 * reader loop, so an RX-only stretch (no TX, no control transaction) reaches
 * disk within about a second instead of at the next TX or a clean close. */
static void log_flush_if_stale(DWORD max_age_ms)
{
    if (!InterlockedCompareExchange(&s_log_dirty, 0, 0)) return;
    if (GetTickCount() - s_log_flushed_at < max_age_ms) return;
    dev_log_flush();
}

void dev_log_stats(const char *when)
{
    if (!logging_enabled()) return;
    dev_log("transport stats (%s): rx_bytes=%ld frames=%ld msg_frames=%ld noise_bytes=%ld "
            "bad_csum=%ld runt=%ld bad_magic=%ld oversize=%ld short_msg=%ld "
            "len_clamped=%ld unknown_chan=%ld queue_full=%ld queue_alloc_failed=%ld "
            "resp_dropped=%ld "
            "resp_stale=%ld resp_unusable=%ld tx_unknown_chan=%ld "
            "xact_timeout=%ld read_err=%ld write_err=%ld "
            "comm_err=%ld",
            when, s_stat.rx_bytes, s_stat.rx_frames, s_stat.rx_messages,
            s_stat.rx_noise_bytes,
            s_stat.rx_bad_csum, s_stat.rx_runt, s_stat.rx_bad_magic,
            s_stat.rx_oversize, s_stat.rx_short_msg, s_stat.rx_len_clamped,
            s_stat.rx_unknown_chan, s_stat.rx_queue_full, s_stat.rx_queue_alloc_failed,
            s_stat.resp_dropped,
            s_stat.resp_stale, s_stat.resp_unusable, s_stat.tx_unknown_chan,
            s_stat.xact_timeout, s_stat.read_errors,
            s_stat.write_errors, s_stat.comm_errors);
    dev_log("repeat stats (%s): no_reply_resends=%ld malformed_ignored=%ld",
            when, s_stat.repeat_no_reply, s_stat.repeat_malformed);
    dev_log("license stats (%s): unlock_attempts=%ld ok=%ld fail=%ld "
            "unlock_ms_total=%ld unlock_ms_max=%ld refresh_run=%ld refresh_skipped=%ld "
            "open_gate_fe=%ld open_recovered=%ld (refresh_ms=%lu)",
            when, s_stat.lic_unlock_attempts, s_stat.lic_unlock_ok, s_stat.lic_unlock_fail,
            s_stat.lic_unlock_ms_total, s_stat.lic_unlock_ms_max,
            s_stat.lic_refresh_run, s_stat.lic_refresh_skipped,
            s_stat.lic_open_gate_fe, s_stat.lic_open_recovered,
            (unsigned long)s_license_refresh_ms);
    dev_log_flush();
}

#include "ini_config.h"

/* Hex-dump a buffer into the diagnostic log.  The old version stopped at 48
 * bytes with nothing to say it had: a multi-frame ISO15765 reply or a 36
 * transfer-data block came out looking like a complete short frame.  Dumps are
 * now whole unless hex_max says otherwise, and always carry their true length. */
static void log_hex(const char *tag, uint8_t chan, const uint8_t *p, int n)
{
    dev_log_hex(p, n, "%s ch=%u len=%d", tag, chan, n);
}

/* One line describing what produced this file, so a log found later can be
 * attributed to a process, a build and a port without external notes. */
static void log_header_locked(void)
{
    char exe[MAX_PATH] = "?", dll[MAX_PATH] = "?";
    GetModuleFileNameA(NULL, exe, sizeof(exe));
    if (s_module) GetModuleFileNameA(s_module, dll, sizeof(dll));
    SYSTEMTIME ts; GetLocalTime(&ts);
    FILE *f = s_logf;
    fprintf(f, "==== OpenVCX log opened %04u-%02u-%02u %02u:%02u:%02u "
               "(local) ====\n", ts.wYear, ts.wMonth, ts.wDay, ts.wHour,
            ts.wMinute, ts.wSecond);
    fprintf(f, "  host process : %s (pid %lu)\n", exe,
            (unsigned long)GetCurrentProcessId());
    fprintf(f, "  DLL module   : %s\n", dll);
    fprintf(f, "  DLL build    : " __DATE__ " " __TIME__ "\n");
    if (s_hex_max > 0)
        fprintf(f, "  log          : %s (level %d, hex dumps capped at %d "
                   "bytes -- capped lines carry shown=)\n", s_logpath,
                s_log_level, s_hex_max);
    else
        fprintf(f, "  log          : %s (level %d, hex dumps complete)\n",
                s_logpath, s_log_level);
    fprintf(f, "  timestamps   : local wall clock, no date; t= is the thread id\n");
    fflush(f);
}

void dev_init(HINSTANCE dll_module)
{
    s_module = dll_module;
    ini_load(dll_module);
    timeBeginPeriod(1);                 /* make the reader's Sleep(1) ~1 ms, not ~15 ms */
    InitializeCriticalSection(&s_io);
    InitializeCriticalSection(&s_chan);
    InitializeCriticalSection(&s_per);
    InitializeCriticalSection(&s_resp_cs);
    InitializeCriticalSection(&s_license);
    InitializeCriticalSection(&s_link);
    if (!s_log_cs_ready) {
        InitializeCriticalSection(&s_log_cs);
        s_log_cs_ready = true;
    }
    s_resp_event = CreateEvent(NULL, FALSE, FALSE, NULL);
    s_cs_ready = true;
    memset(s_channels, 0, sizeof(s_channels));
    s_log_failed = false;
    s_log_header_done = false;
    s_logpath[0] = 0;
    char v[MAX_PATH];
    if (ini_get("log", s_logpath, sizeof(s_logpath))) InterlockedExchange(&s_logging, 1);
    s_hex_max = 256;
    if (ini_get("hex_max", v, sizeof(v))) s_hex_max = atoi(v);
    if (ini_get("log_level", v, sizeof(v))) s_log_level = atoi(v);
    if (ini_get("rx_log_every", v, sizeof(v))) s_rx_log_every = atoi(v);
    s_repeat_reply_timeout_ms = REPEAT_REPLY_TIMEOUT_MS;
    if (ini_get("repeat_reply_timeout_ms", v, sizeof(v)))
        s_repeat_reply_timeout_ms = (DWORD)atoi(v);
    s_fast_init_timeout_ms = 0;
    if (ini_get("fast_init_timeout_ms", v, sizeof(v)))
        s_fast_init_timeout_ms = (DWORD)atoi(v);
#ifdef VCX_RESEARCH_CONFIG
    /* Session pre-refresh age in ms (0 = reactive-only; install once, then let
     * OPEN's 0xFE retry reinstall on demand).  Default preserves the historical
     * 240 s cadence. */
    if (ini_get("license_refresh_ms", v, sizeof(v))) {
        /* Unlike atol(), reject a value with no digits (empty/garbage) instead
         * of silently treating it as 0 -- 0 is a distinct, meaningful setting
         * here (reactive-only), not merely an invalid-input floor like the
         * options above. A typo must fall back to the documented default, not
         * to reactive-only. */
        const char *s = v;
        char *end = NULL;
        long ms = strtol(s, &end, 10);
        s_license_refresh_ms = (end == s || ms < 0) ? VCX_LICENSE_REFRESH_MS : (DWORD)ms;
    }
    if (s_license_refresh_ms == 0) {
        /* Reactive-only has no recovery path for the START/TX gate (only OPEN's
         * 0xFE retry reinstalls a lost session) -- if the session object does not
         * survive indefinitely, a lost session here surfaces only as an
         * unexplained write timeout, not a status this DLL can catch and log.
         * Say so loudly at init so a bench run knows what to watch for instead
         * of debugging a silent timeout blind; see license stats' open_gate_fe. */
        dev_log("license_refresh_ms=0 (reactive-only): periodic PASSTHRU refresh disabled. "
                "OPEN's 0xFE retry can recover a lost session, but the separate START/TX "
                "gate cannot -- a lost session there surfaces only as an unexplained write "
                "timeout, with no catchable status tying it back to licensing.");
    }

    dev_log("configuration: RESEARCH overrides enabled");
    /* Programming-voltage honesty policy (see s_prog_policy / the capability
     * warning block).  Accepts a name (strict|honest|loose|compat, also stock
     * as an alias for compat) or the numeric 0..3; anything else keeps the
     * safe default.  Relax this only if a tool refuses the strict, spec-correct
     * refusals -- e.g. HDS aborting on a boilerplate SetProgrammingVoltage. */
    {
        if (ini_get("voltage_policy", v, sizeof(v))) {
            const char *s = v;
            if      (!_stricmp(s, "strict") || !strcmp(s, "0")) s_prog_policy = PROG_V_STRICT;
            else if (!_stricmp(s, "honest") || !strcmp(s, "1")) s_prog_policy = PROG_V_HONEST;
            else if (!_stricmp(s, "loose")  || !strcmp(s, "2")) s_prog_policy = PROG_V_LOOSE;
            else if (!_stricmp(s, "compat") || !_stricmp(s, "stock") ||
                     !strcmp(s, "3"))                           s_prog_policy = PROG_V_COMPAT;
            else dev_log("voltage_policy=\"%s\" unrecognised; keeping strict", s);
        }
        dev_log("voltage_policy=%s",
                s_prog_policy == PROG_V_STRICT ? "strict" :
                s_prog_policy == PROG_V_HONEST ? "honest" :
                s_prog_policy == PROG_V_LOOSE  ? "loose"  : "compat");
    }

    /* J2534 validation strictness (see dev_strict_validation).  Default 0 keeps
     * the DLL vendor-compatible; 1 turns the spec's rejections on for the inputs
     * the vendor tolerated.  Accepts 0/1 or off/on/lenient/strict. */
    {
        if (ini_get("strict_validation", v, sizeof(v))) {
            const char *s = v;
            if      (!_stricmp(s, "1") || !_stricmp(s, "on") || !_stricmp(s, "true") || !_stricmp(s, "strict"))
                s_strict_validation = true;
            else if (!_stricmp(s, "0") || !_stricmp(s, "off") || !_stricmp(s, "false") || !_stricmp(s, "lenient"))
                s_strict_validation = false;
            else dev_log("strict_validation=\"%s\" unrecognised; keeping lenient", s);
        }
        dev_log("strict_validation=%d", (int)s_strict_validation);
    }

#endif
    dev_log("configuration: validation=%s voltage=%d refresh_ms=%lu keep_warm=%d; restart host to reload",
            s_strict_validation ? "strict" : "HDS-compatible", (int)s_prog_policy,
            (unsigned long)s_license_refresh_ms, keep_warm_enabled());
    for (unsigned i = 0; i < sizeof(s_ini)/sizeof(s_ini[0]); ++i) {
        if (s_ini[i].invalid) dev_log("configuration: invalid %s; using compiled default", s_ini[i].key);
        else dev_log("configuration: %s=%s", s_ini[i].key,
                     *s_ini[i].value ? s_ini[i].value : "<default>");
    }

    /* Validate logging at DLL initialization, immediately after all log
     * configuration has been parsed.  This creates the file/header before any
     * J2534 entry point is called, and makes a bad path observable at load time
     * instead of silently surfacing only on the first later dev_log(). */
    if (logging_enabled()) {
        EnterCriticalSection(&s_log_cs);
        (void)log_file_locked();
        LeaveCriticalSection(&s_log_cs);
    }
}

/* J2534 has no pre-FreeLibrary shutdown export, and warm-link workers must
 * survive PassThruClose. Pin their code before creating any worker. A caller
 * can release its DLL handle without unloading code a worker still executes.
 * This intentionally retains the module until process exit after first use.
 * Called outside DllMain with s_link held. */
static bool pin_worker_module(void)
{
    HMODULE module;
    return GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                             GET_MODULE_HANDLE_EX_FLAG_PIN,
                             (LPCSTR)&s_module, &module) != 0;
}

void dev_shutdown(void)
{
    /* Explicit callers must be outside DllMain when workers exist. The
     * FreeLibrary path can reach this only without workers (module pinning). */
    if (s_handoff_thread) {
        if (s_handoff_quit) SetEvent(s_handoff_quit);
        /* Wait for real exit, not a fixed timeout: handoff_proc can be blocked
         * behind s_link for up to PORT_HANDOFF_WAIT_MS (3000ms) inside a
         * concurrent dev_connect()'s scan, and deleting s_link below out from
         * under a thread that's still going to enter it is undefined behavior. */
        WaitForSingleObject(s_handoff_thread, INFINITE);
        CloseHandle(s_handoff_thread); s_handoff_thread = NULL;
    }
    if (s_handoff_quit) { CloseHandle(s_handoff_quit); s_handoff_quit = NULL; }
    dev_disconnect();
    dev_log_stats("process shutdown");
    if (s_cs_ready) {
        for (int i = 0; i < DEV_MAX_CHANNELS; i++) {
            if (s_channels[i].rx_event) {
                CloseHandle(s_channels[i].rx_event);
                s_channels[i].rx_event = NULL;
            }
        }
        DeleteCriticalSection(&s_io); DeleteCriticalSection(&s_chan);
        DeleteCriticalSection(&s_per); DeleteCriticalSection(&s_resp_cs);
        DeleteCriticalSection(&s_license); DeleteCriticalSection(&s_link);
        if (s_resp_event) CloseHandle(s_resp_event);
        s_cs_ready = false;
        /* A caller can have observed logging enabled just before shutdown.
         * Keep this lock alive for the DLL lifetime so it can recheck the flag
         * instead of entering a deleted critical section. */
        EnterCriticalSection(&s_log_cs);
        InterlockedExchange(&s_logging, 0);
        if (s_logf) { fclose(s_logf); s_logf = NULL; }
        LeaveCriticalSection(&s_log_cs);
    }
    timeEndPeriod(1);
}

/* ---- framing (vcx_transport.c) ---- */
/* Read one BB..BB frame into content (unescaped, checksum-verified). */
static int read_frame(uint8_t *content, int cap, DWORD timeout_ms)
{
    vcx_parser_t p;
    vcx_parser_init(&p, content, cap);
    DWORD start = GetTickCount();
    for (;;) {
        if (GetTickCount() - start > timeout_ms) return -1;
        uint8_t b; DWORD got = 0;
        if (!ReadFile(s_com, &b, 1, &got, NULL) || got == 0) { Sleep(1); continue; }
        switch (vcx_parser_push(&p, b)) {
        case VCX_RX_FRAME:
            return p.len - 1;
        case VCX_RX_BAD_CSUM:
            STAT_BUMP(rx_bad_csum);
            dev_log_hex(content, p.len, "< RX BAD CHECKSUM (sync read) got=%02X "
                        "want=%02X len=%d", content[p.len - 1],
                        vcx_checksum(content, p.len - 1), p.len);
            return -1;
        case VCX_RX_OVERSIZE:
            STAT_LOG(rx_oversize, "< RX DROPPED oversize frame (sync read): "
                     ">%d bytes (#%ld)", p.len);
            return -1;
        default:
            break;
        }
    }
}

/* ---- channel table (wire_id == vcx_chan+1) ---- */
dev_channel_t *dev_channel_add(uint32_t wire_id, uint32_t protocol)
{
    /* wire_id == chan+1 for the slot do_connect already reserved (in_use=true)
     * under s_chan before releasing it -- index directly into that slot rather
     * than re-scanning for "any free slot", which is what let two concurrent
     * connects settle on the same physical channel. */
    if (wire_id == 0 || wire_id > DEV_MAX_CHANNELS) return NULL;
    int i = (int)(wire_id - 1);
    EnterCriticalSection(&s_chan);
    dev_channel_t *c = &s_channels[i];
    HANDLE ev = c->rx_event;
    uint64_t generation = c->generation + 1;
    memset(c, 0, sizeof(*c));
    c->generation = generation;
    c->in_use = true; c->wire_id = wire_id; c->protocol = protocol;
    c->rx_event = ev ? ev : CreateEvent(NULL, FALSE, FALSE, NULL);
    s_filter_seq[i] = 0;
    LeaveCriticalSection(&s_chan);
    return c;
}

static dev_channel_t *chan_find_locked(uint32_t wire_id)
{
    for (int i = 0; i < DEV_MAX_CHANNELS; i++)
        if (s_channels[i].in_use && s_channels[i].wire_id == wire_id) return &s_channels[i];
    return NULL;
}

dev_channel_t *dev_channel_find(uint32_t wire_id)
{
    EnterCriticalSection(&s_chan);
    dev_channel_t *c = chan_find_locked(wire_id);
    LeaveCriticalSection(&s_chan);
    return c;
}

bool dev_channel_snapshot(uint32_t wire_id, dev_channel_snapshot_t *snapshot)
{
    EnterCriticalSection(&s_chan);
    dev_channel_t *c = chan_find_locked(wire_id);
    if (c) {
        snapshot->protocol = c->protocol;
        snapshot->generation = c->generation;
        snapshot->rx_event = c->rx_event;
    }
    LeaveCriticalSection(&s_chan);
    return c != NULL;
}

bool dev_channel_take_overflow(uint32_t wire_id, uint64_t generation, bool *overflow)
{
    EnterCriticalSection(&s_chan);
    dev_channel_t *c = chan_find_locked(wire_id);
    bool valid = c && c->generation == generation;
    if (valid) {
        *overflow = c->overflowed;
        c->overflowed = false;
    }
    LeaveCriticalSection(&s_chan);
    return valid;
}

static void chan_free_locked(dev_channel_t *c)
{
    size_t slot = (size_t)(c - s_channels);
    s_filt_n[slot] = 0;
    memset(s_block[slot], 0, sizeof(s_block[slot]));
    rx_msg_t *m = c->head; while (m) { rx_msg_t *nx = m->next; free(m); m = nx; }
    HANDLE ev = c->rx_event;
    uint64_t generation = c->generation;
    memset(c, 0, sizeof(*c));
    c->generation = generation;
    c->rx_event = ev;
    if (ev) SetEvent(ev);
}

void dev_channel_remove(uint32_t wire_id)
{
    EnterCriticalSection(&s_chan);
    if (wire_id && wire_id <= DEV_MAX_CHANNELS) {
        memset(&s_repeat[wire_id - 1], 0, sizeof(repeat_t));
        repeat_set_awaiting_locked((uint8_t)(wire_id - 1), false);
    }
    dev_channel_t *c = chan_find_locked(wire_id);
    if (c) chan_free_locked(c);
    LeaveCriticalSection(&s_chan);
}

void dev_channels_clear(void)
{
    EnterCriticalSection(&s_chan);
    memset(s_repeat, 0, sizeof(s_repeat));
    for (int i = 0; i < DEV_MAX_CHANNELS; i++) repeat_set_awaiting_locked((uint8_t)i, false);
    for (int i = 0; i < DEV_MAX_CHANNELS; i++) if (s_channels[i].in_use) chan_free_locked(&s_channels[i]);
    LeaveCriticalSection(&s_chan);
}

rx_msg_t *dev_channel_pop(dev_channel_t *ch)
{
    rx_msg_t *m = NULL;
    EnterCriticalSection(&s_chan);
    if (ch->head) { m = ch->head; ch->head = m->next; if (!ch->head) ch->tail = NULL; ch->count--; }
    LeaveCriticalSection(&s_chan);
    return m;
}

long dev_channel_clear_rx(uint32_t wire_id, uint64_t *generation)
{
    EnterCriticalSection(&s_chan);
    dev_channel_t *c = chan_find_locked(wire_id);
    if (!c) { LeaveCriticalSection(&s_chan); return ERR_INVALID_CHANNEL_ID; }
    rx_msg_t *head = c->head;
    int count = c->count;
    c->head = c->tail = NULL;
    c->count = 0;
    c->overflowed = false;
    ResetEvent(c->rx_event);
    if (generation) *generation = c->generation;
    LeaveCriticalSection(&s_chan);
    while (head) { rx_msg_t *next = head->next; free(head); head = next; }
    dev_log("  CLEAR_RX_BUFFER ch=%lu: discarded %d host messages; firmware clear is a stub",
            (unsigned long)wire_id, count);
    return STATUS_NOERROR;
}

bool dev_channel_pop_generation(uint32_t wire_id, uint64_t generation,
                                rx_msg_t **msg, HANDLE *event)
{
    *msg = NULL;
    EnterCriticalSection(&s_chan);
    dev_channel_t *c = chan_find_locked(wire_id);
    if (!c || c->generation != generation) { LeaveCriticalSection(&s_chan); return false; }
    *event = c->rx_event;
    if (c->head) {
        *msg = c->head;
        c->head = c->head->next;
        if (!c->head) c->tail = NULL;
        c->count--;
    }
    LeaveCriticalSection(&s_chan);
    return true;
}

/* ---- reader thread: route RX events, hand control replies to vcx_xact ---- */
/* Append one message to a channel's rx queue. Caller holds s_chan. */
typedef enum {
    CHAN_PUSH_OK,
    CHAN_PUSH_FULL,
    CHAN_PUSH_ALLOC_FAILED,
} chan_push_result_t;

/* Full host QueryPerformanceCounter clock in microseconds.
 * The split conversion keeps count * 1e6 from overflowing on long uptimes. */
static uint64_t host_us64(void)
{
    LARGE_INTEGER freq, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&now);
    return (uint64_t)((now.QuadPart / freq.QuadPart) * 1000000 +
                      (now.QuadPart % freq.QuadPart) * 1000000 / freq.QuadPart);
}

uint32_t dev_host_us(void)
{
    return (uint32_t)host_us64();
}

#define VCX_SERIAL_BAUD 921600u

/* Arrival estimate for byte i of an n-byte read that returned at read_us.
 * Every later byte in the read still had to cross the 8N1 serial line (10 bit
 * times each), so byte i arrived no later than read_us minus their transfer
 * time.  Stamps are kept strictly increasing, so stamp order is the serial
 * arrival order across all channels. Keep ordering in 64 bits so long idle
 * periods cannot be mistaken for a backward clock step. */
static uint32_t rx_byte_stamp(uint64_t read_us, DWORD n, DWORD i, uint64_t *last_us)
{
    uint64_t after_us = (uint64_t)(n - 1 - i) * 10u * 1000000u / VCX_SERIAL_BAUD;
    uint64_t stamp = read_us - after_us;
    if (stamp <= *last_us) stamp = *last_us + 1;
    *last_us = stamp;
    return (uint32_t)stamp;
}

/* The caller reports capacity and allocation failures separately.  Only the
 * former is an application-visible queue overflow. */
static chan_push_result_t chan_push_locked(dev_channel_t *c, uint32_t rx_status,
                                           const uint8_t *data, uint16_t len,
                                           uint32_t stamp_us)
{
    if (c->count >= DEV_RX_QUEUE_CAP) {
        c->overflowed = true;
        return CHAN_PUSH_FULL;
    }
    rx_msg_t *m = (rx_msg_t *)malloc(sizeof(rx_msg_t) + len);
    if (!m) return CHAN_PUSH_ALLOC_FAILED;
    m->next = NULL; m->rx_status = rx_status; m->timestamp = stamp_us;
    m->len = len; if (len) memcpy(m->data, data, len);
    if (c->tail) c->tail->next = m; else c->head = m;
    c->tail = m; c->count++;
    SetEvent(c->rx_event);
    return CHAN_PUSH_OK;
}

/* Protocol a live channel was connected with, or 0 once it is gone. */
static uint32_t chan_protocol(uint32_t wire_id)
{
    EnterCriticalSection(&s_chan);
    dev_channel_t *c = chan_find_locked(wire_id);
    uint32_t proto = c ? c->protocol : 0;
    LeaveCriticalSection(&s_chan);
    return proto;
}

static void route_rx(uint8_t chan, const uint8_t *content, int clen, uint32_t stamp_us)
{
    /* content: 80 <cmd_hi> 00 chan | rxstatus(4 BE) len(2 BE) canid+data(len).
     * The 10-byte floor covers the header the fields below are read from; a
     * shorter frame used to underflow len to ~64 KB and copy that much. */
    if (clen < 10) {
        STAT_LOG_HEX(rx_short_msg, content, clen,
                     "< RX MALFORMED message frame len=%d "
                     "(needs 10 for its header) (#%ld)", clen);
        return;
    }
    uint32_t rxstatus = ((uint32_t)content[4] << 24) | ((uint32_t)content[5] << 16) |
                        ((uint32_t)content[6] << 8) | content[7];
    uint16_t len = (uint16_t)((content[8] << 8) | content[9]);
    uint16_t declared = len;
    if (10 + (int)len > clen) len = (uint16_t)(clen - 10);
    if (len > VCX_MAX_MSG_DATA) len = VCX_MAX_MSG_DATA;
    if (len != declared) {
        /* The frame claims more payload than it carries.  The clamp keeps this
         * from over-reading, but the discrepancy is a firmware or framing
         * signal in its own right and used to be applied silently. */
        STAT_LOG(rx_len_clamped,
                 "  rx ch=%u declared len=%u clamped to %u (frame len=%d) (#%ld)",
                 chan, declared, len, clen);
    }
    EnterCriticalSection(&s_chan);
    dev_channel_t *c = chan_find_locked((uint32_t)chan + 1);
    bool had_channel = (c != NULL);
    bool blocked = false;
    chan_push_result_t push = CHAN_PUSH_OK;
    if (c) {
        if (rx_blocked_locked(chan, &content[10], len)) {
            blocked = true;   /* BLOCK_FILTER match: drop before it reaches the app */
            /* ...but it was still the ECU's turn on a half-duplex wire. */
            repeat_rx_locked(c, chan, rxstatus, &content[10], len, false);
        } else {
            repeat_rx_locked(c, chan, rxstatus, &content[10], len, len == declared);
            push = chan_push_locked(c, rxstatus, &content[10], len, stamp_us);
        }
    }
    LeaveCriticalSection(&s_chan);
    if (blocked) {
        if (dev_log_enabled())
            dev_log("  rx ch=%u dropped by BLOCK_FILTER (len=%u)", chan, len);
    } else if (!had_channel) {
        /* Traffic on a channel this DLL never opened is exactly the artifact
         * worth keeping -- and was the one thing thrown away without a word. */
        STAT_LOG(rx_unknown_chan,
                 "  rx DROPPED: no open channel for wire ch=%u "
                 "(rxstatus=0x%08lX len=%u) (#%ld)", chan,
                 (unsigned long)rxstatus, len);
    } else if (push == CHAN_PUSH_FULL) {
        STAT_LOG(rx_queue_full,
                 "  rx DROPPED: channel %u receive queue full at %d messages "
                 "(#%ld)", chan, DEV_RX_QUEUE_CAP);
    } else if (push == CHAN_PUSH_ALLOC_FAILED) {
        STAT_LOG(rx_queue_alloc_failed,
                 "  rx DROPPED: channel %u receive queue allocation FAILED "
                 "(len=%u) (#%ld)", chan, len);
    }
}

/* Hold every control reply until the transaction waiter consumes it.  The old
 * single slot was overwritten by whichever reply landed next, so two replies
 * arriving inside one timeout window lost the first.  SetEvent is
 * deliberately outside the lock: the queue is authoritative and the
 * auto-reset event is only a prompt to drain it.
 *
 * A full queue drops its OLDEST entry instead of refusing the new one.  s_io
 * serializes control transactions, so at most one reply is ever legitimately
 * outstanding; a queue this deep means the device is spraying frames, and the
 * newest is then the likeliest to be the one somebody is waiting for.  Losing
 * a stale frame is a non-event -- costing the caller its transaction over it
 * would not be. */
static void queue_control_response(const uint8_t *content, int clen)
{
    bool dropped = false;
    if (clen < 0 || clen > (int)sizeof(s_resp_queue[0].data)) {
        STAT_BUMP(resp_dropped);
        dev_log("control reply DROPPED: len=%d exceeds the %d-byte reply slot",
                clen, (int)sizeof(s_resp_queue[0].data));
        return;
    }
    EnterCriticalSection(&s_resp_cs);
    if (s_resp_count == VCX_RESP_QUEUE_CAP) {
        s_resp_head = (s_resp_head + 1) % VCX_RESP_QUEUE_CAP;
        s_resp_count--;
        dropped = true;
    }
    control_response_t *r = &s_resp_queue[s_resp_tail];
    r->len = clen;
    memcpy(r->data, content, (size_t)clen);
    s_resp_tail = (s_resp_tail + 1) % VCX_RESP_QUEUE_CAP;
    s_resp_count++;
    LeaveCriticalSection(&s_resp_cs);
    SetEvent(s_resp_event);
    if (dropped) {
        STAT_BUMP(resp_dropped);
        dev_log("control response queue full (cap %d); dropped oldest reply",
                VCX_RESP_QUEUE_CAP);
    }
}

/* Copies only the bytes a reply actually occupies -- a control reply is a
 * handful of bytes, and the slot it sits in is sized for the 4 KB worst case. */
static bool dequeue_control_response(control_response_t *out)
{
    bool have = false;
    EnterCriticalSection(&s_resp_cs);
    if (s_resp_count != 0) {
        const control_response_t *r = &s_resp_queue[s_resp_head];
        out->len = r->len;
        memcpy(out->data, r->data, (size_t)r->len);
        s_resp_head = (s_resp_head + 1) % VCX_RESP_QUEUE_CAP;
        s_resp_count--;
        have = true;
    }
    LeaveCriticalSection(&s_resp_cs);
    return have;
}

/* Called while s_io excludes any other transaction.  Anything queued before
 * this request's write cannot be its reply. */
static void discard_queued_control_responses(void)
{
    EnterCriticalSection(&s_resp_cs);
    s_resp_head = s_resp_tail = s_resp_count = 0;
    ResetEvent(s_resp_event);
    LeaveCriticalSection(&s_resp_cs);
}

static void handle_frame(const uint8_t *content, int clen, uint32_t stamp_us)
{
    /* Log before the validity gates, not after: a frame this DLL rejects is
     * precisely the one worth having in the file.  Both of these used to
     * return without leaving any trace at all. */
    if (clen < 5) {
        STAT_LOG_HEX(rx_runt, content, clen,
                     "< RX MALFORMED runt len=%d (#%ld)", clen);
        return;
    }
    if (content[0] != 0x80) {
        STAT_LOG_HEX(rx_bad_magic, content, clen,
                     "< RX MALFORMED magic=%02X want=80 len=%d (#%ld)",
                     content[0], clen);
        return;
    }
    STAT_BUMP(rx_frames);
    uint8_t cmd_lo = content[2], chan = content[3];
    LONG message_n = cmd_lo == VCX_OP_MSG ? STAT_BUMP(rx_messages) : 0;
    /* A busy CAN bus can otherwise turn one 64 KB log buffer over for every
     * few frames.  Control replies remain complete; vehicle frames retain the
     * first 64 and then a marked sample, while stats report the true total. */
    if (cmd_lo != VCX_OP_MSG || rx_log_sample(message_n))
        dev_log_hex(content, clen, "< RX ch=%u len=%d sample=%ld", chan, clen,
                    message_n);
    /* On the receive side, cmd_lo==0 is a message (RX event / TxDone echo, seen
     * with cmd_hi 0 or 1); any nonzero cmd_lo is a control reply. */
    if (cmd_lo == VCX_OP_MSG) {
        route_rx(chan, content, clen, stamp_us);
    } else {
        queue_control_response(content, clen);
    }
}

/* Block-read the port and stream-parse BB..BB frames. Reading in chunks (not a
 * byte per syscall) is essential: a live CAN bus floods RX events, and a slow
 * reader would let control-command replies drown / the OS buffer overflow. */
static DWORD WINAPI reader_proc(LPVOID arg)
{
    (void)arg;
    static uint8_t buf[2048], frame[4300];
    vcx_parser_t rx;
    vcx_parser_init(&rx, frame, (int)sizeof(frame));
    uint64_t last_stamp_us = host_us64();
    while (InterlockedCompareExchange(&s_reader_run, 1, 1)) {
        DWORD got = 0;
        log_flush_if_stale(1000);
        if (!ReadFile(s_com, buf, sizeof(buf), &got, NULL)) {
            /* A failed read and an idle port used to take the same branch, so
             * an unplugged device or a driver fault was indistinguishable from
             * a quiet bus -- the log simply went silent. */
            DWORD gle = GetLastError();
            LONG n = STAT_BUMP(read_errors);
            DWORD ce = 0; COMSTAT cs; memset(&cs, 0, sizeof(cs));
            bool have_state = ClearCommError(s_com, &ce, &cs) != 0;
            if (ce) STAT_BUMP(comm_errors);
            /* Tell a transient read failure from a link that is gone.  The
             * device vanishing (unplug, USB re-enumeration, a driver reset)
             * leaves a handle that fails every read forever: the old code
             * Sleep(1)ed and retried, so one unplug cost 4096 logged failures
             * over 8 s before the next PassThruOpen noticed the handle was
             * stale.  ClearCommError failing too means the handle no longer
             * refers to a port at all, which is the unambiguous signal; on top
             * of that, err=5/6 on a read of a serial handle is never transient.
             * Stop the reader and let the handoff watcher close the port, so
             * the dead handle stops locking every other process out of it. */
            bool fatal = !have_state || gle == ERROR_ACCESS_DENIED ||
                         gle == ERROR_INVALID_HANDLE || gle == ERROR_BAD_COMMAND ||
                         gle == ERROR_DEVICE_REMOVED || gle == ERROR_DEV_NOT_EXIST;
            if (fatal) {
                dev_log("< RX READ FAILED err=%lu commerr=0x%lX (#%ld): the link "
                        "is gone; stopping the reader and dropping the port",
                        (unsigned long)gle, (unsigned long)ce, n);
                InterlockedExchange(&s_link_dead, 1);
                break;
            }
            if (stat_verbose(n))
                dev_log("< RX READ FAILED err=%lu commerr=0x%lX inqueue=%lu (#%ld)",
                        (unsigned long)gle, (unsigned long)ce,
                        (unsigned long)cs.cbInQue, n);
            Sleep(1); continue;
        }
        if (got == 0) { Sleep(1); continue; }
        uint64_t read_us = host_us64();
        InterlockedExchangeAdd(&s_stat.rx_bytes, (LONG)got);
        for (DWORD i = 0; i < got; i++) {
            switch (vcx_parser_push(&rx, buf[i])) {
            case VCX_RX_FRAME:
                /* i is the frame's closing delimiter. */
                handle_frame(frame, rx.len - 1,
                             rx_byte_stamp(read_us, got, i, &last_stamp_us));
                break;
            case VCX_RX_BAD_CSUM:
                STAT_LOG_HEX(rx_bad_csum, frame, rx.len,
                             "< RX BAD CHECKSUM got=%02X "
                             "want=%02X len=%d (#%ld)", frame[rx.len - 1],
                             vcx_checksum(frame, rx.len - 1), rx.len);
                break;
            case VCX_RX_OVERSIZE:
                STAT_LOG(rx_oversize,
                         "< RX DROPPED oversize frame: >%d bytes, "
                         "reassembly buffer is %d (#%ld)",
                         rx.len, (int)sizeof(frame));
                break;
            case VCX_RX_NOISE:
                /* Bytes ahead of the first delimiter: a mid-frame attach, or
                 * the device talking before we synchronized. Counted, not lost. */
                STAT_BUMP(rx_noise_bytes);
                break;
            case VCX_RX_NONE:
                break;
            }
        }
    }
    dev_log("reader thread exit");
    dev_log_stats("reader exit");
    return 0;
}

/* Log a WriteFile failure and bump write_errors. prefix identifies the call
 * ("xact op=%02X ch=%u" or "tx ch=%u"); gle must be the GetLastError() value
 * captured immediately after the failing WriteFile. */
static void log_write_failed(const char *prefix, DWORD wrote, int flen, DWORD gle)
{
    LONG n = STAT_BUMP(write_errors);
    dev_log("  %s WRITE FAILED: %lu of %d bytes, err=%lu (#%ld)",
            prefix, (unsigned long)wrote, flen, (unsigned long)gle, n);
}

/* Control transaction: write frame, wait for its reply. Returns device status
 * (>=0) or -1 on transport failure. */
static long vcx_xact(uint8_t cmd_hi, uint8_t opcode, uint8_t chan,
                     const uint8_t *payload, int plen,
                     uint8_t *resp, uint16_t *resp_len, uint16_t cap)
{
#ifdef VCX_CONFIG_TEST
    return config_test_xact(opcode, chan, payload, plen);
#endif
    if (s_com == INVALID_HANDLE_VALUE) {
        dev_log("  xact op=%02X ch=%u ABORTED: port not open", opcode, chan);
        return -1;
    }
    uint8_t frame[VCX_MAX_FRAME];
    int flen = vcx_frame_build(cmd_hi, opcode, chan, payload, plen, frame, sizeof(frame));
    if (flen < 0) {
        dev_log_hex(payload, plen, "  xact op=%02X ch=%u ABORTED: payload len=%d "
                    "does not fit a frame", opcode, chan, plen);
        return -1;
    }

    DWORD t_start = GetTickCount();
    EnterCriticalSection(&s_io);
    long rc = -1;
    bool matched_reply = false;
    discard_queued_control_responses();
    DWORD wrote = 0;
    /* Formatted inside s_io, immediately before the write.  If it were logged
     * outside, two threads racing for the port could record frames in the opposite
     * order to the one they reached the wire in -- and the file's whole value
     * is that its order is the wire's order. */
    log_hex("> TX", chan, payload, plen);
    BOOL wok = WriteFile(s_com, frame, flen, &wrote, NULL);
    if (!wok || wrote != (DWORD)flen) {
        DWORD gle = GetLastError();
        char prefix[32]; snprintf(prefix, sizeof(prefix), "xact op=%02X ch=%u", opcode, chan);
        log_write_failed(prefix, wrote, flen, gle);
    }
    if (wok && wrote == (DWORD)flen) {
        /* A reply is ours only if BOTH the opcode and the channel byte match.
         * Anything else is a straggler from a transaction that already gave
         * up: discard it and keep waiting out the remaining budget rather than
         * mistaking it for this request's answer. */
        DWORD start = GetTickCount();
        for (;;) {
            DWORD elapsed = GetTickCount() - start;
            if (elapsed >= DEV_REQ_TIMEOUT_MS) break;
            if (WaitForSingleObject(s_resp_event, DEV_REQ_TIMEOUT_MS - elapsed) != WAIT_OBJECT_0)
                break;
            control_response_t queued;
            while (dequeue_control_response(&queued)) {
                int clen = queued.len;
                const uint8_t *data = queued.data;
                if (clen < 5 || data[0] != 0x80) {
                    LONG n = STAT_BUMP(resp_unusable);
                    dev_log_hex(data, clen, "  xact op=%02X ch=%u: discarding "
                                "unusable queued reply len=%d (#%ld)", opcode,
                                chan, clen, n);
                    continue;
                }
                if (data[2] != opcode || data[3] != chan) {
                    STAT_BUMP(resp_stale);
                    dev_log_hex(data, clen, "  xact op=%02X ch=%u: discarding "
                                "stale reply op=%02X ch=%u len=%d", opcode, chan,
                                data[2], data[3], clen);
                    continue;
                }
                rc = data[4];
                if (resp) {
                    int extra = clen - 5; if (extra < 0) extra = 0;
                    if (extra > cap) {
                        /* The caller's buffer is smaller than the reply.  The
                         * bytes past it are dropped from the return value, so
                         * the whole frame goes in the log instead of vanishing. */
                        dev_log_hex(&data[5], clen - 5, "  xact op=%02X ch=%u reply "
                                    "TRUNCATED to caller: %d of %d extra bytes; full "
                                    "reply len=%d", opcode, chan, cap, extra, clen);
                        extra = cap;
                    }
                    memcpy(resp, &data[5], extra); if (resp_len) *resp_len = (uint16_t)extra;
                }
                matched_reply = true;
                break;
            }
            if (matched_reply) break;
        }
    }
    /* If this device ever does answer late, the evidence is a frame still
     * sitting in the queue at the moment we gave up.  Record it rather than
     * act on it: the one capture of this failure mode shows the request being
     * dropped outright, and that reading should stay checkable from a log
     * alone if the firmware ever behaves differently. */
    if (wok && wrote == (DWORD)flen && !matched_reply) {
        unsigned leftover;
        EnterCriticalSection(&s_resp_cs);
        leftover = s_resp_count;
        LeaveCriticalSection(&s_resp_cs);
        STAT_BUMP(xact_timeout);
        dev_log("  xact op=%02X ch=%u timed out after %lu ms (%u unclaimed reply "
                "frame(s) queued)", opcode, chan,
                (unsigned long)(GetTickCount() - t_start), leftover);
    }
    LeaveCriticalSection(&s_io);
    /* Preserve TX-line ordering with the write while keeping the disk sync
     * out of s_io. */
    dev_log_flush();
    /* Elapsed covers the wait for s_io as well as the device's answer: a slow
     * transaction that was actually queued behind another thread reads very
     * differently from a slow device, and the timestamps alone cannot tell
     * them apart. */
    dev_log("  xact op=%02X ch=%u -> rc=%ld in %lu ms", opcode, chan, rc,
            (unsigned long)(GetTickCount() - t_start));
    return rc;
}

/* One message handed back to the application after its transmit reaches the
 * wire: the J2534 loopback echo and/or the ISO15765 TxDone indication. */
typedef struct { uint32_t rx_status; const uint8_t *data; uint16_t len; } tx_confirm_t;

/* Message TX.  The device sends nothing back for a transmit -- no reply, and no
 * TxDone frame in any vendor capture -- but J2534-1 04.04 has the interface
 * queue the application's own transmit back to it: a full echo while LOOPBACK
 * is on, and, on ISO15765, a TxDone indication.  Apps read those before the
 * ECU's answer, so do_write builds them and they are queued here while the
 * write is still in flight.  Holding s_chan across the write is what keeps them
 * ahead of the answer: the reader thread has to take s_chan to deliver it, so
 * it cannot interleave.  s_io is taken first, the lock order used everywhere
 * else in this file. */
static long vcx_send_msg(uint8_t chan, const uint8_t *payload, int plen,
                         const tx_confirm_t *confirm, int nconfirm)
{
    if (s_com == INVALID_HANDLE_VALUE) {
        dev_log("  tx ch=%u ABORTED: port not open", chan);
        return -1;
    }
    uint8_t frame[VCX_MAX_FRAME];
    int flen = vcx_frame_build(0x01, VCX_OP_MSG, chan, payload, plen, frame, sizeof(frame));
    if (flen < 0) {
        dev_log_hex(payload, plen, "  tx ch=%u ABORTED: payload len=%d does not "
                    "fit a frame", chan, plen);
        return -1;
    }
    EnterCriticalSection(&s_io);
    DWORD wrote = 0;
    /* Inside s_io for the same reason as vcx_xact: the log's order is only
     * worth anything if it is the order the frames left the host. */
    log_hex("> TXMSG", chan, payload, plen);
    EnterCriticalSection(&s_chan);
    BOOL wok = WriteFile(s_com, frame, flen, &wrote, NULL);
    long rc = (wok && wrote == (DWORD)flen) ? STATUS_NOERROR : -1;
    if (rc != STATUS_NOERROR) {
        DWORD gle = GetLastError();
        char prefix[24]; snprintf(prefix, sizeof(prefix), "tx ch=%u", chan);
        log_write_failed(prefix, wrote, flen, gle);
    }
    if (rc == STATUS_NOERROR && nconfirm > 0) {
        uint32_t stamp_us = dev_host_us();
        dev_channel_t *c = chan_find_locked((uint32_t)chan + 1);
        int lost_full = 0, lost_alloc = 0;
        for (int i = 0; c && i < nconfirm; i++) {
            chan_push_result_t push = chan_push_locked(c, confirm[i].rx_status,
                                                        confirm[i].data, confirm[i].len,
                                                        stamp_us);
            if (push == CHAN_PUSH_FULL) lost_full++;
            else if (push == CHAN_PUSH_ALLOC_FAILED) lost_alloc++;
        }
        if (!c) {
            LONG n = InterlockedExchangeAdd(&s_stat.tx_unknown_chan, nconfirm) +
                     nconfirm;
            dev_log("  tx ch=%u: no open channel for %d transmit confirmation(s) "
                    "DROPPED (total #%ld)", chan, nconfirm, n);
        }
        else if (lost_full) {
            LONG n = InterlockedExchangeAdd(&s_stat.rx_queue_full, lost_full) + lost_full;
            if (stat_verbose(n))
                dev_log("  tx ch=%u: %d of %d transmit confirmation(s) DROPPED "
                        "(receive queue full; total #%ld)", chan, lost_full,
                        nconfirm, n);
        }
        if (c && lost_alloc) {
            LONG n = InterlockedExchangeAdd(&s_stat.rx_queue_alloc_failed, lost_alloc) + lost_alloc;
            if (stat_verbose(n))
                dev_log("  tx ch=%u: %d of %d transmit confirmation(s) DROPPED "
                        "(receive queue allocation FAILED; total #%ld)", chan,
                        lost_alloc, nconfirm, n);
        }
    }
    LeaveCriticalSection(&s_chan);
    LeaveCriticalSection(&s_io);
    /* The line was formatted under s_io, immediately before WriteFile.  Flush
     * only after releasing that lock so slow storage cannot delay the wire. */
    dev_log_flush();
    return rc;
}

/* ---- transient PASSTHRU license/session ----
 *
 * Firmware 1.9.4.2 derives its 16-byte outer key as:
 *   GetInfo.raw[16:24] || DH_shared_secret_le64
 * where DH uses g=5 over p=2^64-59.  It does not reject the public value 1,
 * therefore shared_secret=1 for every device private exponent.  The outer
 * cipher is corrected Block TEA with 64+floor(52/n) rounds (not standard
 * XXTEA's 6+floor(52/n)).  The only trailer check is A567A567 at plaintext end.
 *
 * The Nano already contains the signed entitlement database.  A1 merely
 * selects the PASSTHRU record; it does not upload or forge a license.
 */
static uint32_t load32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void store32le(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16); p[3] = (uint8_t)(value >> 24);
}

static uint32_t xxtea_mix(uint32_t z, uint32_t y, uint32_t sum,
                          const uint32_t key[4], unsigned index, unsigned selector)
{
    return (((z >> 5) ^ (y << 2)) + ((y >> 3) ^ (z << 4))) ^
           ((y ^ sum) + (z ^ key[(index & 3u) ^ selector]));
}

static bool xxtea_crypt(uint8_t *data, size_t length, const uint8_t key_bytes[16], bool decrypt)
{
    uint32_t words[40], key[4];
    if (length < 8 || (length & 3u) != 0 || length > sizeof(words)) return false;
    size_t count = length / 4;
    for (size_t i = 0; i < count; i++) words[i] = load32le(data + i * 4);
    for (size_t i = 0; i < 4; i++) key[i] = load32le(key_bytes + i * 4);

    uint32_t rounds = 64u + 52u / (uint32_t)count;
    if (!decrypt) {
        uint32_t sum = 0, z = words[count - 1];
        for (uint32_t round = 0; round < rounds; round++) {
            sum += VCX_XXTEA_DELTA;
            unsigned selector = (sum >> 2) & 3u;
            for (size_t i = 0; i + 1 < count; i++) {
                uint32_t y = words[i + 1];
                words[i] += xxtea_mix(z, y, sum, key, (unsigned)i, selector);
                z = words[i];
            }
            uint32_t y = words[0];
            words[count - 1] += xxtea_mix(z, y, sum, key, (unsigned)(count - 1), selector);
            z = words[count - 1];
        }
    } else {
        uint32_t sum = rounds * VCX_XXTEA_DELTA, y = words[0];
        while (sum != 0) {
            unsigned selector = (sum >> 2) & 3u;
            for (size_t i = count - 1; i > 0; i--) {
                uint32_t z = words[i - 1];
                words[i] -= xxtea_mix(z, y, sum, key, (unsigned)i, selector);
                y = words[i];
            }
            uint32_t z = words[count - 1];
            words[0] -= xxtea_mix(z, y, sum, key, 0, selector);
            y = words[0];
            sum -= VCX_XXTEA_DELTA;
        }
    }
    for (size_t i = 0; i < count; i++) store32le(data + i * 4, words[i]);
    return true;
}

static bool license_seal(const uint8_t *plain, size_t plain_len,
                         const uint8_t key[16], uint8_t *sealed, size_t sealed_cap)
{
    if (plain_len + 16 > sealed_cap || ((plain_len + 16) & 3u) != 0) return false;
    memcpy(sealed, plain, plain_len);
    memset(sealed + plain_len, 0, 16);
    store32le(sealed + plain_len, VCX_OUTER_MARKER);
    FILETIME now; GetSystemTimeAsFileTime(&now);
    memcpy(sealed + plain_len + 4, &now, sizeof(now));
    store32le(sealed + plain_len + 12, GetTickCount());
    return xxtea_crypt(sealed, plain_len + 16, key, false);
}

static bool license_open(uint8_t *sealed, size_t sealed_len, size_t plain_len,
                         const uint8_t key[16])
{
    return sealed_len == plain_len + 16 && xxtea_crypt(sealed, sealed_len, key, true) &&
           load32le(sealed + plain_len) == VCX_OUTER_MARKER;
}

/* Fold one completed handshake's already-measured wall-time (>= 0) into the
 * license stats. Takes the elapsed ms rather than re-measuring it so a caller
 * can reuse the same sample for both this and its own log line, instead of
 * two GetTickCount() calls disagreeing by a few ms. */
static void license_record_duration(LONG ms)
{
    InterlockedExchangeAdd(&s_stat.lic_unlock_ms_total, ms);
    LONG prev;
    do { prev = s_stat.lic_unlock_ms_max; }
    while (ms > prev &&
           InterlockedCompareExchange(&s_stat.lic_unlock_ms_max, ms, prev) != prev);
}

static bool license_unlock(void)
{
    if (!s_have_license_key_prefix || s_com == INVALID_HANDLE_VALUE) return false;
    /* Serialize the whole 5-transaction handshake: do_connect() and
     * chan_bring_up() can each reach this on their own thread, and without a
     * lock spanning the full sequence their DH/QUERY/SET/GET steps could
     * interleave on the wire. */
    EnterCriticalSection(&s_license);
    STAT_BUMP(lic_unlock_attempts);
    DWORD unlock_start = GetTickCount();
    uint8_t key[16] = {0}, public_one[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    memcpy(key, s_license_key_prefix, 8);
    key[8] = 1;                         /* DH shared secret, little-endian */

    uint8_t device_public[8]; uint16_t response_len = 0;
    long status = vcx_xact(0, VCX_OP_LICENSE_DH, 0, public_one, sizeof(public_one),
                           device_public, &response_len, sizeof(device_public));
    if (status != VCX_STATUS_OK || response_len != 8) goto fail;

    uint8_t query[16] = {'P','A','S','S','T','H','R','U', 0,0,0,0,0,0,0,0};
    uint8_t sealed_query[32], record[32];
    if (!license_seal(query, sizeof(query), key, sealed_query, sizeof(sealed_query))) goto fail;
    response_len = 0;
    status = vcx_xact(0, VCX_OP_LICENSE_QUERY, 0, sealed_query, sizeof(sealed_query),
                      record, &response_len, sizeof(record));
    if (status != VCX_STATUS_OK || !license_open(record, response_len, 16, key) ||
        memcmp(record, "PASSTHRU", 8) != 0) goto fail;

    /* Firmware generates a new private exponent for the second phase.  Public
     * value 1 keeps the newly derived shared secret equal to 1 as well. */
    response_len = 0;
    status = vcx_xact(0, VCX_OP_LICENSE_DH, 0, public_one, sizeof(public_one),
                      device_public, &response_len, sizeof(device_public));
    if (status != VCX_STATUS_OK || response_len != 8) goto fail;

    uint8_t selector[0x90] = {0}, sealed_selector[0xA0];
    store32le(selector + 0x80, 0);       /* accept the device record's own validity */
    store32le(selector + 0x84, VCX_PASSTHRU_ID);
    memcpy(selector + 0x88, "PASSTHRU", 8);
    if (!license_seal(selector, sizeof(selector), key, sealed_selector,
                      sizeof(sealed_selector))) goto fail;
    status = vcx_xact(0, VCX_OP_LICENSE_SET, 0, sealed_selector, sizeof(sealed_selector),
                      NULL, NULL, 0);
    if (status != VCX_STATUS_OK) goto fail;

    if (!license_seal(query, sizeof(query), key, sealed_query, sizeof(sealed_query))) goto fail;
    response_len = 0;
    status = vcx_xact(0, VCX_OP_LICENSE_GET, 0, sealed_query, sizeof(sealed_query),
                      record, &response_len, sizeof(record));
    if (status != VCX_STATUS_OK || !license_open(record, response_len, 16, key) ||
        memcmp(record, "PASSTHRU", 8) != 0) goto fail;

    s_license_tick = GetTickCount();
    STAT_BUMP(lic_unlock_ok);
    LONG elapsed_ms = (LONG)(s_license_tick - unlock_start);
    if (elapsed_ms < 0) elapsed_ms = 0;
    license_record_duration(elapsed_ms);
    dev_log("PASSTHRU session established directly (record=%02X%02X%02X%02X flags=%02X%02X%02X%02X, took %lu ms)",
            record[8], record[9], record[10], record[11],
            record[12], record[13], record[14], record[15],
            (unsigned long)elapsed_ms);
    LeaveCriticalSection(&s_license);
    return true;

fail: {
    STAT_BUMP(lic_unlock_fail);
    LONG elapsed_ms = (LONG)(GetTickCount() - unlock_start);
    if (elapsed_ms < 0) elapsed_ms = 0;
    license_record_duration(elapsed_ms);
    dev_log("PASSTHRU session handshake failed (last status=%ld, response_len=%u, took %lu ms)",
            status, (unsigned)response_len, (unsigned long)elapsed_ms);
    LeaveCriticalSection(&s_license);
    return false;
}
}

static bool license_is_fresh(void)
{
    if (s_license_tick == 0) return false;        /* no session installed yet */
    if (s_license_refresh_ms == 0) return true;   /* reactive-only: trust the install + 0xFE retry */
    return (DWORD)(GetTickCount() - s_license_tick) < s_license_refresh_ms;
}

/* ---- port discovery (synchronous; reader thread not yet running) ---- */
static bool sync_is_vcx(void)
{
    uint8_t frame[16];
    int flen = vcx_frame_build(0x00, VCX_OP_GETINFO, 0, NULL, 0, frame, sizeof(frame));
    DWORD wrote = 0;
    if (!WriteFile(s_com, frame, flen, &wrote, NULL) || wrote != (DWORD)flen) return false;
    uint8_t content[256];
    int clen = read_frame(content, sizeof(content), 800);
    if (clen < 8) return false;
    for (int i = 0; i + 8 <= clen; i++)
        if (memcmp(&content[i], "VCX-NANO", 8) == 0) {
            /* response = four-byte header, status, then 64-byte raw info */
            if (clen >= 29 && content[4] == VCX_STATUS_OK) {
                memcpy(s_license_key_prefix, content + 5 + 16, 8);
                s_have_license_key_prefix = true;
            }
            if (clen >= 5 + (int)sizeof(s_devinfo) && content[4] == VCX_STATUS_OK) {
                memcpy(s_devinfo, content + 5, sizeof(s_devinfo));
                s_have_devinfo = true;
                dev_log_hex(s_devinfo, (int)sizeof(s_devinfo), "GETINFO block len=%d",
                            (int)sizeof(s_devinfo));
            }
            return true;
        }
    return false;
}

/* Log a failed Win32 comm-port setup call. Must be called immediately after
 * the failing call, before any other Win32 call clobbers GetLastError().
 * always logs regardless of log_level; otherwise the message is gated like
 * other low-severity setup diagnostics. */
static void log_comm_fail(const char *name, const char *what, bool always)
{
    if (always || dev_log_verbose())
        dev_log("port %s: %s err=%lu", name, what, (unsigned long)GetLastError());
}

/* One attempt at one port.  Leaves the CreateFile status in s_last_open_err so
 * try_open can tell "no such port" from "somebody else has it". */
static bool try_open_once(const char *name)
{
    char path[32]; snprintf(path, sizeof(path), "\\\\.\\%s", name);
    HANDLE h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    s_last_open_err = (h == INVALID_HANDLE_VALUE) ? GetLastError() : 0;
    if (h == INVALID_HANDLE_VALUE) {
        /* A failed CreateFile used to log nothing at all, which is why a port
         * held by another process was indistinguishable in the log from a port
         * that does not exist: the scan simply never mentioned it.  Absence is
         * the ordinary case and stays quiet at level 1; anything else is said
         * out loud, because it is the difference between "plug the Nano in" and
         * "another program has it". */
        if (s_last_open_err == ERROR_ACCESS_DENIED) {
            /* Quiet inside the handoff poll loop: sixty identical lines say
             * nothing the one line before and the one after do not. */
            if (!s_open_quiet)
                dev_log("port %s: exists but is open in another process "
                        "(CreateFile err=%lu)", name,
                        (unsigned long)s_last_open_err);
        } else if (s_last_open_err != ERROR_FILE_NOT_FOUND) {
            /* GetLastError() is still s_last_open_err here -- nothing Win32-side
             * has run since it was captured above. */
            log_comm_fail(name, "CreateFile failed", false);
        }
        return false;
    }
    DCB dcb; memset(&dcb, 0, sizeof(dcb)); dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) log_comm_fail(name, "GetCommState failed", false);
    dcb.BaudRate = VCX_SERIAL_BAUD; dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE; dcb.fDtrControl = DTR_CONTROL_ENABLE; dcb.fRtsControl = RTS_CONTROL_ENABLE;
    /* An unchecked SetCommState leaves the link at whatever the port was last
     * set to -- a silent wrong-baud session that looks like a dead device. */
    if (!SetCommState(h, &dcb))
        log_comm_fail(name, "SetCommState(921600 8N1) FAILED", true);
    if (!SetupComm(h, 65536, 8192))
        log_comm_fail(name, "SetupComm failed", false);  /* large RX buffer to absorb CAN-bus floods */
    COMMTIMEOUTS to = {0};
    /* Non-blocking reads: return immediately with whatever is buffered. (A
     * blocking ReadFile can't be used here because the handle is synchronous, so
     * a blocked read would serialize against WriteFile on the command thread.)
     * The reader's idle Sleep(1) is kept honest by timeBeginPeriod(1) in
     * dev_init, so it wakes in ~1 ms instead of a ~15 ms timer tick -- that tick
     * of latency was slow enough that FORScan's short UDS read timeout missed
     * every ECU response. */
    to.ReadIntervalTimeout = MAXDWORD; to.ReadTotalTimeoutConstant = 0; to.ReadTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 500;
    /* A failed SetCommTimeouts leaves reads blocking, which deadlocks the
     * reader against the command thread on this synchronous handle. */
    if (!SetCommTimeouts(h, &to))
        log_comm_fail(name, "SetCommTimeouts FAILED", true);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    s_com = h;
    if (sync_is_vcx()) {
        /* Read the link settings back rather than reprint what we asked for:
         * a USB-serial driver may coerce baud, handshaking or buffer sizes,
         * and the effective values are what explain a marginal link. */
        DCB eff; memset(&eff, 0, sizeof(eff)); eff.DCBlength = sizeof(eff);
        COMMTIMEOUTS et; memset(&et, 0, sizeof(et));
        BOOL have_dcb = GetCommState(h, &eff), have_to = GetCommTimeouts(h, &et);
        dev_log("port %s: VCX-NANO identified, using it", name);
        if (have_dcb)
            dev_log("  link: %lu baud %u%c%s binary=%d dtr=%d rts=%d "
                    "xon/xoff=%d/%d cts=%d dsr=%d",
                    (unsigned long)eff.BaudRate, eff.ByteSize,
                    eff.Parity == NOPARITY ? 'N' : eff.Parity == ODDPARITY ? 'O' :
                    eff.Parity == EVENPARITY ? 'E' : eff.Parity == MARKPARITY ? 'M' : 'S',
                    eff.StopBits == ONESTOPBIT ? "1" :
                    eff.StopBits == TWOSTOPBITS ? "2" : "1.5",
                    eff.fBinary, eff.fDtrControl, eff.fRtsControl,
                    eff.fOutX, eff.fInX, eff.fOutxCtsFlow, eff.fOutxDsrFlow);
        else
            dev_log("  link: settings unreadable, err=%lu",
                    (unsigned long)GetLastError());
        if (have_to)
            dev_log("  link timeouts: read interval=%lu total=%lu+%lu*n, "
                    "write total=%lu+%lu*n (rx buffer 65536, tx 8192)",
                    (unsigned long)et.ReadIntervalTimeout,
                    (unsigned long)et.ReadTotalTimeoutConstant,
                    (unsigned long)et.ReadTotalTimeoutMultiplier,
                    (unsigned long)et.WriteTotalTimeoutConstant,
                    (unsigned long)et.WriteTotalTimeoutMultiplier);
        DWORD ce = 0; COMSTAT cs; memset(&cs, 0, sizeof(cs));
        if (ClearCommError(h, &ce, &cs) && (ce || cs.cbInQue || cs.cbOutQue))
            dev_log("  link state at open: commerr=0x%lX inqueue=%lu outqueue=%lu",
                    (unsigned long)ce, (unsigned long)cs.cbInQue,
                    (unsigned long)cs.cbOutQue);
        handoff_publish(name);
        return true;
    }
    /* Which ports were opened but did not answer is half of a "device not
     * found" diagnosis, and none of it used to reach the log. */
    if (dev_log_verbose())
        dev_log("port %s: opened but no VCX-NANO GETINFO reply", name);
    CloseHandle(h); s_com = INVALID_HANDLE_VALUE; return false;
}

/* ---- cross-process port handoff ---- */

/* The name lives in the caller's session namespace rather than the global one:
 * HDS and the helpers it launches run as one user in one session, and the
 * session namespace needs no privilege a per-user install may not have. */
static void handoff_event_name(const char *port, char *out, size_t cap)
{
    snprintf(out, cap, "Local\\OpenVCX_Release_%s", port);
}

/* The holder's answer channel.  Without it a refusal is indistinguishable from
 * a holder that is wedged, asleep or gone: both look like ACCESS_DENIED, and
 * the requester can only tell them apart by waiting out its whole ceiling.
 * Being patient is right for the second case and wrong for the first -- HDS
 * hands the Nano between processes on a ~300 ms cadence and needs the wait, but
 * a tool whose user genuinely has the device open in another window deserves
 * "it is in use" now, not seventeen seconds from now. */
static void handoff_busy_event_name(const char *port, char *out, size_t cap)
{
    snprintf(out, cap, "Local\\OpenVCX_Busy_%s", port);
}

/* Announce that this process holds the port.  Manual-reset, so a request
 * raised between two watcher polls is still there when the watcher looks.
 *
 * The reset below is load-bearing, not defensive.  CreateEvent on a name that
 * already exists returns a handle to the EXISTING object and *ignores* the
 * initial-state argument, and a requester only ever SetEvent()s -- it leaves
 * the reset to the holder.  So a request that went unserviced (holder wedged,
 * killed, or its watcher gone) leaves the object signaled forever, and the next
 * process to claim the port inherits it that way: it publishes, its watcher
 * fires on the next 100 ms tick, and it drops the link it just spent four
 * seconds discovering -- with no requester anywhere.  That is exactly the
 * unexplained "release requested by another process" one poll after a
 * successful open, and the full COM1-32 rescan behind it. */
static void handoff_publish(const char *port)
{
    char nm[64];
    handoff_event_name(port, nm, sizeof(nm));
    if (s_release_evt) CloseHandle(s_release_evt);
    s_release_evt = CreateEventA(NULL, TRUE, FALSE, nm);
    if (!s_release_evt) {
        dev_log("port %s: cannot publish the handoff event (err=%lu); other "
                "processes will not be able to ask for this port", port,
                (unsigned long)GetLastError());
    } else {
        /* GetLastError() is only meaningful immediately after CreateEvent. */
        bool inherited = (GetLastError() == ERROR_ALREADY_EXISTS);
        if (inherited && WaitForSingleObject(s_release_evt, 0) == WAIT_OBJECT_0)
            dev_log("port %s: inherited a stale signaled handoff event from a "
                    "previous holder; clearing it", port);
        ResetEvent(s_release_evt);
    }
    handoff_busy_event_name(port, nm, sizeof(nm));
    if (s_busy_evt) CloseHandle(s_busy_evt);
    s_busy_evt = CreateEventA(NULL, TRUE, FALSE, nm);
    /* Same inherited-state hazard as the release event above, and the same
     * cure: start clear, so a refusal left over from a previous holder is never
     * read as this one's. */
    if (s_busy_evt) ResetEvent(s_busy_evt);
    snprintf(s_port_name, sizeof(s_port_name), "%s", port);
}

static void handoff_withdraw(void)
{
    /* If another handle to this named event is still briefly open elsewhere
     * (e.g. a requester between its SetEvent and CloseHandle), our CloseHandle
     * alone won't destroy the kernel object -- reset it first so a later
     * handoff_publish() on this same port name can't inherit it signaled. */
    if (s_release_evt) { ResetEvent(s_release_evt); CloseHandle(s_release_evt); s_release_evt = NULL; }
    if (s_busy_evt) { ResetEvent(s_busy_evt); CloseHandle(s_busy_evt); s_busy_evt = NULL; }
    s_port_name[0] = 0;
}

/* The port is taken.  If the holder is another OpenVCX32.dll it published the
 * event above, so ask it to let go and take the port the moment it does.  A
 * port held by something that is not one of ours has no event to open, and is
 * left alone without waiting. */
static bool handoff_wait_and_open(const char *name)
{
    char nm[64];
    handoff_event_name(name, nm, sizeof(nm));
    HANDLE e = OpenEventA(EVENT_MODIFY_STATE, FALSE, nm);
    if (!e) {
        /* ACCESS_DENIED with no OpenVCX handoff event: something else has the
         * port and there's nobody to ask -- still busy, not absent, so say so
         * rather than letting the scan fall through to "not found". */
        dev_log("port %s: held by a non-OpenVCX process; not waiting", name);
        s_connect_busy = true;
        snprintf(s_busy_port, sizeof(s_busy_port), "%s", name);
        return false;
    }
    /* The holder's reply channel.  Opened for SYNCHRONIZE|MODIFY so we can both
     * clear a stale refusal and wait on a fresh one; absent (older holder, or
     * the object not yet created) simply means no answer is coming and we fall
     * back to waiting the request out. */
    char bnm[64];
    handoff_busy_event_name(name, bnm, sizeof(bnm));
    HANDLE busy = OpenEventA(SYNCHRONIZE | EVENT_MODIFY_STATE, FALSE, bnm);
    /* Clear before asking: only a refusal raised in answer to *this* request
     * may cut the wait short. */
    if (busy) ResetEvent(busy);
    dev_log("port %s: held by another OpenVCX process; asking it to release",
            name);
    dev_log_flush();
    SetEvent(e);
    s_open_quiet = true;
    DWORD t0 = GetTickCount(), waited = 0;
    for (;;) {
        Sleep(PORT_HANDOFF_POLL_MS);
        waited = GetTickCount() - t0;
        if (try_open_once(name)) {
            CloseHandle(e);
            if (busy) CloseHandle(busy);
            s_open_quiet = false;
            dev_log("port %s: released after %lu ms", name,
                    (unsigned long)waited);
            return true;
        }
        /* An explicit refusal ends the wait immediately.  Waiting is for a
         * holder that has not answered; a holder that says "I have a session
         * open" has answered, and sitting out the ceiling on top of that just
         * delays the honest ERR_DEVICE_IN_USE the caller needs to see. */
        if (busy && WaitForSingleObject(busy, 0) == WAIT_OBJECT_0) {
            CloseHandle(e); CloseHandle(busy);
            s_open_quiet = false;
            dev_log("port %s: the holder refused after %lu ms -- it has a J2534 "
                    "session open", name, (unsigned long)waited);
            s_connect_busy = true;
            snprintf(s_busy_port, sizeof(s_busy_port), "%s", name);
            return false;
        }
        /* Re-assert the request on every poll rather than once up front.  The
         * event is manual-reset and shared by name, so a holder that publishes
         * the port in this same window (handoff_publish clears an inherited
         * event) would otherwise wipe a live request and leave us waiting out
         * the whole ceiling for a release nobody is still being asked for. */
        SetEvent(e);
        /* Keep waiting whatever the port says.  We are only here because this
         * port answered CreateFile with ACCESS_DENIED *and* had an OpenVCX
         * handoff event published on it, so it is the Nano -- and the states it
         * passes through on the way back are not all ACCESS_DENIED.  The old
         * code broke out of the loop on anything else, which meant the two ways
         * a release actually completes both looked like give-up conditions:
         * FILE_NOT_FOUND while the USB bridge re-enumerates, and
         * "opened but no GETINFO reply" while it reboots.  The elapsed-time
         * ceiling is what bounds this, not the error code. */
        if (waited >= PORT_HANDOFF_WAIT_MS) break;
    }
    CloseHandle(e);
    if (busy) CloseHandle(busy);
    s_open_quiet = false;
    dev_log("port %s: still held after %lu ms (last open err=%lu) -- the other "
            "process is not letting go", name, (unsigned long)waited,
            (unsigned long)s_last_open_err);
    s_connect_busy = true;
    snprintf(s_busy_port, sizeof(s_busy_port), "%s", name);
    return false;
}

static bool try_open(const char *name)
{
    if (try_open_once(name)) return true;
    if (s_last_open_err != ERROR_ACCESS_DENIED) return false;
    return handoff_wait_and_open(name);
}

/* Hands the port to whoever asked for it, and closes a link the reader has
 * declared dead.  Runs on its own thread for the life of the DLL: releasing is
 * a dev_disconnect, which joins the reader, so the reader cannot do it. */
static DWORD WINAPI handoff_proc(LPVOID arg)
{
    (void)arg;
    dev_log("handoff watcher started (tick %d ms)", PORT_HANDOFF_WATCH_MS);
    for (;;) {
        /* Only WAIT_OBJECT_0 means "quit".  The old loop condition was
         * `== WAIT_TIMEOUT`, which also exited on WAIT_FAILED -- so a single
         * failed wait retired the watcher permanently, with nothing in the log
         * to say so, and this process then held the port until it exited: every
         * later release request and every reader-declared dead link went
         * unserviced in silence.  Treat a failure as a failure. */
        DWORD w = WaitForSingleObject(s_handoff_quit, PORT_HANDOFF_WATCH_MS);
        if (w == WAIT_OBJECT_0) break;
        if (w != WAIT_TIMEOUT) {
            dev_log("handoff watcher: wait returned %lu (err=%lu); retrying",
                    (unsigned long)w, (unsigned long)GetLastError());
            dev_log_flush();
            Sleep(PORT_HANDOFF_WATCH_MS);
            continue;
        }
        /* Cheap peek before paying for s_link: the common tick, especially
         * once fully disconnected, has nothing to do. A stale read here just
         * costs one extra 100ms poll, never a missed release/dead-link. */
        if (!InterlockedCompareExchange(&s_link_dead, 0, 0) && !s_release_evt)
            continue;
        EnterCriticalSection(&s_link);
        if (InterlockedCompareExchange(&s_link_dead, 0, 0) &&
            s_com != INVALID_HANDLE_VALUE) {
            dev_log("link declared dead by the reader; closing the port");
            dev_disconnect();
        }
        HANDLE e = s_release_evt;
        if (e && WaitForSingleObject(e, 0) == WAIT_OBJECT_0) {
            if (s_session_active) {
                /* Refusing is the honest answer, and saying so beats letting
                 * the newcomer time out with no explanation on either side. */
                dev_log("port %s: release requested, but this process has a "
                        "J2534 session open; keeping the port", s_port_name);
                /* Say it on the wire too, not just in this log: the requester
                 * is otherwise staring at a bare ACCESS_DENIED and cannot tell
                 * a refusal from a holder that has stopped answering. */
                if (s_busy_evt) SetEvent(s_busy_evt);
                ResetEvent(e);
            } else {
                dev_log("port %s: release requested by another process; "
                        "dropping the warm link", s_port_name);
                dev_disconnect();   /* clears s_release_evt */
            }
            dev_log_flush();
        }
        LeaveCriticalSection(&s_link);
    }
    dev_log("handoff watcher exiting");
    dev_log_flush();
    return 0;
}

static bool dev_connect_locked(char *err, size_t err_len)
{
    s_connect_busy = false;
    s_busy_port[0] = 0;
    if (s_com != INVALID_HANDLE_VALUE) {
        /* A handle kept warm across a previous PassThruClose.  Confirm the port
         * still exists -- the Nano may have been unplugged while closed -- with
         * a cheap query that fails on a dead handle, and that the reader hasn't
         * already condemned the link (GetCommState can still succeed on a
         * handle the reader gave up on).  If either says the link is gone, drop
         * it and fall through to a full rediscovery so a replug on a new COM
         * number still recovers. */
        DCB probe; memset(&probe, 0, sizeof(probe)); probe.DCBlength = sizeof(probe);
        if (!InterlockedCompareExchange(&s_link_dead, 0, 0) && GetCommState(s_com, &probe))
            return true;
        dev_log("warm COM handle is stale; rediscovering");
        dev_disconnect();
    }
    s_have_license_key_prefix = false;
    s_have_devinfo = false;
    s_license_tick = 0;
    char pin[16];
    const char *env = ini_get("port", pin, sizeof(pin)) ? pin : NULL;
    bool ok = false;
    /* First-open dance: opening any COM port with DTR/RTS asserted resets the
     * Nano's USB-serial bridge, and the device takes ~1-2s to re-answer.  Our
     * per-port GETINFO probe only waits 800 ms, so a first pass can iterate
     * past COM5 while the device is still rebooting.  Retry the whole scan
     * once after a short settle window rather than making users hit Connect
     * twice. */
    dev_log("dev_connect: discovering (preferred port=%s, keep_warm=%d)",
            env && *env ? env : "<unset>",
            keep_warm_enabled() ? 1 : 0);
    /* Within one pass, s_connect_busy short-circuits the remaining fixed
     * candidates: a port that answered the handoff event but would not let go
     * *is* the Nano, so trying env, then ini, then COM5 against the same busy
     * device just repeats the wait.  The COM1-32 sweep deliberately does NOT
     * stop on busy -- a busy hit there only proves *that* port is occupied, and
     * a second physical Nano (or a stale OpenVCX process) further down the list
     * would otherwise never be tried.
     *
     * The second pass, however, now runs even after a busy first pass, with the
     * busy flag cleared first.  Busy is a statement about one moment, not about
     * the device: the holder that refused at t=0 has very often let go by the
     * time the first pass ends, and HDS hands the Nano between processes on
     * roughly that cadence.  Refusing to look again turned a transient overlap
     * into a hard ERR_DEVICE_IN_USE that the application reports as "device is
     * busy, please disconnect first". */
    for (int pass = 0; pass < 2 && !ok; pass++) {
        if (pass) {
            if (s_connect_busy)
                dev_log("dev_connect: %s was busy on the first pass; settling "
                        "and trying once more", s_busy_port);
            s_connect_busy = false;
            s_busy_port[0] = 0;
            Sleep(750);
        }
        ok = (env && *env && try_open(env)) ||
             (!s_connect_busy && try_open("COM5"));
        for (int i = 1; i <= 32 && !ok; i++) {
            char name[8]; snprintf(name, sizeof(name), "COM%d", i); ok = try_open(name);
        }
    }
    if (!ok) {
        if (s_connect_busy) {
            dev_log("dev_connect: %s is held by another process that would not "
                    "release it", s_busy_port);
            if (err && err_len) snprintf(err, err_len,
                "VCX Nano on %s is in use by another process", s_busy_port);
        } else {
            dev_log("dev_connect: no VCX Nano found after 2 passes over COM1-32");
            if (err && err_len) snprintf(err, err_len,
                "VCX Nano not found (set VCX_NANO_PORT or attach on COM5)");
        }
        return false;
    }
    /* Pin before starting any background worker; never join one in DllMain. */
    if (!pin_worker_module()) {
        if (err && err_len) snprintf(err, err_len, "cannot retain worker module");
        dev_disconnect();
        return false;
    }
    /* start the reader thread now that the port is confirmed */
    InterlockedExchange(&s_link_dead, 0);
    InterlockedExchange(&s_reader_run, 1);
    s_reader = CreateThread(NULL, 0, reader_proc, NULL, 0, NULL);
    if (s_reader) {
        /* Eagerly install the transient session.  A failed attempt is not fatal
         * here: do_connect retries if OPEN reports the precise 0xFE gate status. */
        (void)license_unlock();
    }
    /* An unplug can make OFF impossible. Retry it after rediscovery, before
     * returning a usable session, rather than leaving API recovery blocked. */
    if (s_prog_maybe_on && release_prog_voltage() != STATUS_NOERROR) {
        if (err && err_len) snprintf(err, err_len, "programming voltage cleanup unconfirmed");
        dev_disconnect();
        return false;
    }
    /* One watcher for the life of the DLL.  Started here rather than in
     * dev_init because dev_init runs under the loader lock, where creating a
     * thread is not safe. */
    if (!s_handoff_thread) {
        if (!s_handoff_quit) s_handoff_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
        s_handoff_thread = CreateThread(NULL, 0, handoff_proc, NULL, 0, NULL);
        if (!s_handoff_thread)
            dev_log("could not start the handoff watcher (err=%lu); this "
                    "process will hold the port until it exits",
                    (unsigned long)GetLastError());
    }
    return true;
}

bool dev_connect(char *err, size_t err_len, bool *was_busy)
{
    EnterCriticalSection(&s_link);
    bool ok = dev_connect_locked(err, err_len);
    /* From here until dev_session_close the handoff watcher must refuse to give
     * the port away: this process is the one using it. */
    if (ok) s_session_active = true;
    /* A refusal describes the session starting now, never the last one. */
    if (s_busy_evt) ResetEvent(s_busy_evt);
    /* Read s_connect_busy for THIS call before releasing s_link -- a concurrent
     * dev_connect() is otherwise free to reset it before the caller looks. */
    if (was_busy) *was_busy = s_connect_busy;
    LeaveCriticalSection(&s_link);
    return ok;
}

void dev_disconnect(void)
{
    /* Recursive by design: the handoff watcher and dev_connect both already
     * hold s_link when they call this. */
    EnterCriticalSection(&s_link);
    /* Stop advertising the port before closing it, so a process that opens the
     * event a moment from now does not signal a holder that no longer exists. */
    handoff_withdraw();
    s_session_active = false;
    if (s_com != INVALID_HANDLE_VALUE) {
        /* Final driver-side view of the link: overrun/framing/parity flags and
         * anything still sitting in the queues when the port went away. */
        DWORD ce = 0; COMSTAT cs; memset(&cs, 0, sizeof(cs));
        if (ClearCommError(s_com, &ce, &cs))
            dev_log("dev_disconnect: closing the port (commerr=0x%lX inqueue=%lu "
                    "outqueue=%lu)", (unsigned long)ce,
                    (unsigned long)cs.cbInQue, (unsigned long)cs.cbOutQue);
        else
            dev_log("dev_disconnect: closing the port (comm state unreadable, "
                    "err=%lu)", (unsigned long)GetLastError());
        dev_log_stats("port close");
    }
    long voltage_rc = (s_com != INVALID_HANDLE_VALUE || s_prog_maybe_on)
                      ? release_prog_voltage() : STATUS_NOERROR;
    long periodic_rc = periodic_stop_all();
    if (voltage_rc != STATUS_NOERROR || periodic_rc != STATUS_NOERROR)
        dev_log("physical disconnect: cleanup unconfirmed (voltage=%ld timers=%ld)",
                voltage_rc, periodic_rc);
    if (s_reader) {
        InterlockedExchange(&s_reader_run, 0);
        /* Also unblock a read if the driver failed to apply nonblocking
         * timeouts. Never discard a live reader's handle and reuse its state. */
        CancelSynchronousIo(s_reader);
        WaitForSingleObject(s_reader, INFINITE);
        CloseHandle(s_reader); s_reader = NULL;
    }
    dev_channels_clear();
    if (s_com != INVALID_HANDLE_VALUE) {
        /* Exclude any in-flight vcx_xact()/vcx_send_msg() transaction, which
         * holds s_io across its WriteFile(s_com,...) and the reply wait --
         * closing the handle out from under one is a race on Win32. */
        EnterCriticalSection(&s_io);
        CloseHandle(s_com); s_com = INVALID_HANDLE_VALUE;
        LeaveCriticalSection(&s_io);
    }
    s_have_license_key_prefix = false;
    s_have_devinfo = false;
    s_license_tick = 0;
    InterlockedExchange(&s_link_dead, 0);
    LeaveCriticalSection(&s_link);
}

/* PassThruClose keeps the physical link warm by default.  Reopening the port
 * re-asserts DTR/RTS, which resets the Nano's USB-serial bridge and costs ~4 s
 * to re-answer -- the bulk of the 4.85 s median open->connect measured over the
 * 108 opens in notes/honda_hds_non_fail_analysis_2026-09-01.md.  Holding the
 * port, reader thread and licence across a logical close collapses that for
 * every reopen after the first.  The handle is opened non-shared, so while it
 * is held no *other* process can open the device; a warm link is therefore
 * given up on request -- see the handoff protocol above, which is what lets HDS
 * pass the Nano from testman.exe to DataListClient.exe and back.  Bench setups
 * that would rather never hold the port at all can restore the close-fully
 * behaviour with keep_warm=0 in vcx_nano.ini. */
static bool keep_warm_enabled(void)
{
    char v[8];
    if (!ini_get("keep_warm", v, sizeof(v))) return true;   /* default: warm */
    return strcmp(v, "0") && _stricmp(v, "off") && _stricmp(v, "false") && _stricmp(v, "lenient");
}

long dev_session_close(void)
{
    EnterCriticalSection(&s_link);
    /* Keep ownership on failure: the caller can retry Close, and the watcher
     * must not hand off a device with unconfirmed voltage/timer cleanup. */
    long voltage_rc = (s_com != INVALID_HANDLE_VALUE || s_prog_maybe_on)
                      ? release_prog_voltage() : STATUS_NOERROR;
    long timer_rc = periodic_stop_all();
    long rc = voltage_rc != STATUS_NOERROR ? voltage_rc : timer_rc;
    if (rc != STATUS_NOERROR) {
        LeaveCriticalSection(&s_link);
        return rc;
    }
    /* Logical close only: keep the port warm, but still stop and close any
     * device channel the app left open.  A well-behaved app (HDS, FORScan) has
     * already called PassThruDisconnect, so this snapshot is usually empty;
     * without the physical reopen there is no device reset to clear a sloppy
     * app's leftovers otherwise.  Snapshot the open slots under s_chan, then
     * issue the teardown outside it -- vcx_xact takes s_io, and the lock order
     * is s_io before s_chan. */
    uint8_t open_chans[DEV_MAX_CHANNELS]; int n_open = 0;
    EnterCriticalSection(&s_chan);
    for (int i = 0; i < DEV_MAX_CHANNELS; i++)
        if (s_channels[i].in_use) open_chans[n_open++] = (uint8_t)i;
    LeaveCriticalSection(&s_chan);
    for (int i = 0; i < n_open; i++) {
        vcx_xact(0x00, VCX_OP_STOP,  open_chans[i], NULL, 0, NULL, NULL, 0);
        long st = vcx_xact(0x00, VCX_OP_CLOSE, open_chans[i], NULL, 0, NULL, NULL, 0);
        if (st == VCX_STATUS_OK) dev_channel_remove((uint32_t)open_chans[i] + 1);
        else if (rc == STATUS_NOERROR) rc = st < 0 ? -1 : ERR_FAILED;
    }
    if (rc != STATUS_NOERROR) {
        LeaveCriticalSection(&s_link);
        return rc;
    }
    /* The reader keeps draining the wire; with no channels, stray frames are
     * dropped and any queued control reply is flushed by the next xact. */
    dev_channels_clear();
    s_session_active = false;
    if (s_busy_evt) ResetEvent(s_busy_evt);
    if (!keep_warm_enabled()) dev_disconnect();
    LeaveCriticalSection(&s_link);
    return STATUS_NOERROR;
}

bool dev_is_connected(void)
{
    /* s_com is only ever written under s_link (dev_connect_locked,
     * dev_disconnect); an unlocked read here could observe a stale value
     * mid-teardown, since s_com isn't cleared until the end of dev_disconnect. */
    EnterCriticalSection(&s_link);
    bool ok = (s_com != INVALID_HANDLE_VALUE);
    LeaveCriticalSection(&s_link);
    return ok;
}

void dev_api_lock(void) { EnterCriticalSection(&s_link); }
void dev_api_unlock(void) { LeaveCriticalSection(&s_link); }

/* ---- per-channel J2534 config cache (GET answered locally; matches vendor) ---- */
typedef struct { uint32_t param, value; } cfg_kv;
static cfg_kv s_cfg[DEV_MAX_CHANNELS][40];
static int    s_cfg_n[DEV_MAX_CHANNELS];
/* J1962 pins last pushed to the device, so a repeated SET_CONFIG does not
 * restart a channel that is already on the right bus (a restart clears the
 * channel's filters). */
static uint32_t s_pins[DEV_MAX_CHANNELS];
static bool     s_pins_set[DEV_MAX_CHANNELS];
/* J1962 pin a UART/K-line channel is driving, as PID_BUS_PIN's pin1 field.
 * Seeded at connect by uart_pin_default() because the firmware's UART engines
 * default this field to 0, not 7. */
static uint8_t  s_uart_pin[DEV_MAX_CHANNELS];

uint8_t dev_channel_uart_pin(uint32_t wire_id)
{
    return (wire_id && wire_id <= DEV_MAX_CHANNELS) ? s_uart_pin[wire_id - 1] : 0;
}
/* PassThruConnect Flags for the channel.  Channel-level, so a pin-change reopen
 * (SET_CONFIG repin) must replay their firmware-param mappings alongside the
 * cached comm-params -- a fresh open resets every comm-param. */
static uint32_t s_connect_flags[DEV_MAX_CHANNELS];

static uint32_t rd32le(const uint8_t *p)
{ return (uint32_t)p[0] | (p[1]<<8) | (p[2]<<16) | ((uint32_t)p[3]<<24); }

static void wr32le(uint8_t *p, uint32_t v)
{ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }

static void cfg_set(int ch, uint32_t param, uint32_t value)
{
    for (int i = 0; i < s_cfg_n[ch]; i++)
        if (s_cfg[ch][i].param == param) { s_cfg[ch][i].value = value; return; }
    if (s_cfg_n[ch] < 40) { s_cfg[ch][s_cfg_n[ch]].param = param;
                            s_cfg[ch][s_cfg_n[ch]].value = value; s_cfg_n[ch]++; }
}
/* J2534-1 04.04 gives P1_MAX (the longest gap allowed between bytes of an ECU
 * reply) the range 1..0xFFFF and a 20 ms default.  Connect never pushes it, so
 * the cache used to answer GET_CONFIG with 0.  HDS reads P1_MAX before a repeat
 * DTC clear, sets 20 for the clear and writes the saved value back after STOP;
 * that 0 reached the firmware as a 0 us inter-byte limit and no multi-byte
 * reply survived until the channel was reopened (live runs R001-R003: 11
 * restores, 246 writes, 0 replies -- HDS read the silence as ignition off). */
#define KLINE_P1_MAX_DEFAULT_MS 20u

static uint32_t cfg_get(int ch, uint32_t param)
{
    for (int i = 0; i < s_cfg_n[ch]; i++) if (s_cfg[ch][i].param == param) return s_cfg[ch][i].value;
    switch (param) {
    case J2534_CFG_P1_MAX: return KLINE_P1_MAX_DEFAULT_MS;
    case VCX_CFG_ISO15765_TXSTMIN: return 500;
    case VCX_CFG_ISO15765_WAIT_MULT: return 10;
    case VCX_CFG_ISO15765_TIMEOUT_US: return 100000;
    default: return 0;
    }
}

/* PID 0x0400 is consumed by uart_hw_set_frame_format as three packed bytes.
 * J2534 exposes data bits and parity separately and has no stop-bit parameter,
 * so retain the firmware's normal one-stop-bit format. */
static uint32_t uart_format_value(int ch)
{
    uint32_t data_bits = cfg_get(ch, J2534_CFG_DATA_BITS);
    uint32_t parity = cfg_get(ch, J2534_CFG_PARITY);
    /* J2534-1 04.04 encodes 0 = 8 data bits, 1 = 7 data bits; the firmware wants
     * the literal count.  0 is also the never-set default, which is 8 either
     * way.  A literal 7 or 8 is passed through as-is. */
    if (data_bits == 0) data_bits = 8;
    else if (data_bits == 1) data_bits = 7;
    return (data_bits & 0xFFu) | ((parity & 0xFFu) << 8) | (1u << 16);
}
/* J2534 config-param id -> VCX 16-bit param id (0 = cache-only, not pushed).
 * LOOPBACK stays cache-only on purpose. The firmware's bus_can_set_loopback is
 * a controller self-test mode, not the app-level echo J2534 means, so do_write
 * implements LOOPBACK host-side by echoing the transmit back to the app.
 * J1962_PINS is handled separately in do_ioctl: it expands to two device
 * params, so it cannot be expressed as a single id here. */
static uint16_t j2534_to_vcx_param(uint16_t engine, uint32_t p)
{
    /* A shared numeric PID does not imply shared semantics across engines. */
    if (p == VCX_CFG_ISO15765_EXT || p == VCX_CFG_ISO15765_TXBS ||
        p == VCX_CFG_ISO15765_TXSTMIN || p == VCX_CFG_ISO15765_WAIT_MULT ||
        p == VCX_CFG_ISO15765_TIMEOUT_US)
        return engine_is_iso15765(engine) ? (uint16_t)p : 0;
    if (p == J2534_CFG_DATA_RATE) return VCX_PID_BAUDRATE;
    if (p == J2534_CFG_NODE_ADDRESS)
        return (engine == 0x8505 || engine == 0x8606) ? VCX_PID_J1850_NODE_ADDR : 0;
    if (p == J2534_CFG_ISO15765_BS || p == J2534_CFG_ISO15765_STMIN)
        return engine_is_iso15765(engine) ?
            (p == J2534_CFG_ISO15765_BS ? VCX_PID_ISO15765_BS : VCX_PID_ISO15765_STMIN) : 0;
    if (!engine_is_kline(engine)) return 0;
    if (engine == 0x9D04 && p >= J2534_CFG_TIDLE && p <= J2534_CFG_TWUP) return 0;
    if (engine == 0x9204 && (p == J2534_CFG_TWUP || p == J2534_CFG_TINIL)) return 0;
    if (engine_has_slow_init(engine)) {
        switch (p) {
        case J2534_CFG_W1:           return VCX_PID_SLOW_W1_MAX;
        case J2534_CFG_W2:           return VCX_PID_SLOW_W2_MAX;
        case J2534_CFG_W3:           return VCX_PID_SLOW_W3_MAX;
        case J2534_CFG_W4:           return VCX_PID_SLOW_W4_MIN;
        default: break;
        }
        if (engine == 0x9204) {
            /* UART_ECHO_BYTE_PS names the same windows T1..T3. */
            if (p == J2534_2_CFG_UEB_T1_MAX) return VCX_PID_SLOW_W1_MAX;
            if (p == J2534_2_CFG_UEB_T2_MAX) return VCX_PID_SLOW_W2_MAX;
            if (p == J2534_2_CFG_UEB_T3_MAX) return VCX_PID_SLOW_W3_MAX;
        }
    }
    switch (p) {
    case J2534_CFG_P1_MAX:         return VCX_PID_ISO_P1_MAX;
    case J2534_CFG_P2_MAX:         return VCX_PID_ISO_P2_MAX;
    case J2534_CFG_P3_MIN:         return VCX_PID_ISO_P3_MIN;
    case J2534_CFG_P4_MIN:         return VCX_PID_ISO_P4_MIN;
    case J2534_CFG_TIDLE:          return VCX_PID_ISO_TIDLE;
    case J2534_CFG_TWUP:           return VCX_PID_ISO_TWUP;
    case J2534_CFG_TINIL:          return VCX_PID_ISO_TINIL;
    default:                       return 0;
    }
}

/* j2534_to_vcx_param() picks the device PID; this picks the device's units.
 * Every mapped param passes through unscaled except the fast-init timing
 * group, which J2534 expresses in milliseconds and the firmware wants in
 * microseconds (see VCX_PID_ISO_P3_MIN above). */
static uint32_t j2534_to_vcx_value(uint32_t p, uint32_t val)
{
    switch (p) {
    case J2534_CFG_P1_MAX:
    case J2534_CFG_P2_MAX:
    case J2534_CFG_P3_MIN:
    case J2534_CFG_P4_MIN:
    case J2534_CFG_TIDLE:
    case J2534_CFG_TWUP:
    case J2534_CFG_TINIL:
    case J2534_CFG_W1:
    case J2534_CFG_W2:
    case J2534_CFG_W3:
    case J2534_CFG_W4:
    case J2534_2_CFG_UEB_T1_MAX:
    case J2534_2_CFG_UEB_T2_MAX:
    case J2534_2_CFG_UEB_T3_MAX:
        return val * 1000u;
    default:
        return val;
    }
}

/* Append one [param u16 BE][value u32 BE] entry to an opcode-0x45 payload. */
static void par_emit(uint8_t *blob, int *bn, int cap, uint16_t vid, uint32_t val)
{
    if (*bn + 6 > cap) return;
    blob[(*bn)++] = (uint8_t)(vid >> 8);  blob[(*bn)++] = (uint8_t)vid;
    blob[(*bn)++] = (uint8_t)(val >> 24); blob[(*bn)++] = (uint8_t)(val >> 16);
    blob[(*bn)++] = (uint8_t)(val >> 8);  blob[(*bn)++] = (uint8_t)val;
}

/* Tell a UART/K-line engine which J1962 pin it's on -- pin1 in the top byte,
 * pin2 always 0 (see the PID_BUS_PIN writeup below). A no-op pin of 0 means
 * the channel hasn't claimed one yet, so nothing is sent. */
static void emit_kline_pin(uint8_t *blob, int *bn, int cap, uint8_t pin)
{
    if (pin) par_emit(blob, bn, cap, VCX_PID_BUS_PIN, (uint32_t)pin << 24);
}

/* Translate the channel-level PassThruConnect Flags into firmware comm-params,
 * engine-aware.  ISO9141_NO_CHECKSUM turns off the UART engine's own checksum so
 * the application owns it. KW1281 has no checksum PID.
 * ISO9141_K_LINE_ONLY, and the device-private
 * LISTEN_ONLY/SELF_TEST, have no comm-param on a Nano (it is K-line-only on pin
 * 7 regardless) and are decoded for the log only.  Emits nothing for a flag it
 * does not map, so an unset flag leaves the engine's own default. */
static void emit_connect_flag_params(uint8_t *blob, int *bn, int cap,
                                     uint16_t engine, uint32_t flags)
{
    /* Checksum is pushed in BOTH directions rather than only when the app asks
     * to own it.  Leaving it unset would depend on the engine constructor
     * defaulting the field to 1, and this firmware is already known not to be
     * trustworthy about UART engine defaults -- its pin field comes out of the
     * constructor as 0 rather than 7, which is why do_connect has to claim the
     * pin explicitly.  If the checksum field defaults the same way, an ordinary
     * ISO9141/ISO14230 connect would silently get checksum-off behaviour and
     * every response would fail the application's own check. */
    if (engine_is_kline(engine) && engine != 0x9204)
        par_emit(blob, bn, cap, VCX_PID_UART_CHECKSUM,
                 (flags & J2534_CONNECT_ISO9141_NO_CHECKSUM) ? 0u : 1u);
    /* 0x0043 is P3MIN, not CAN ID acceptance. 0x0023 on ISO-TP controls
     * receive indications, not ID width. ID width is carried by messages and
     * filters; do not corrupt timing with a guessed connect-flag mapping. */
}

/* The only two J1962 pin pairs this device can actually reach.
 *
 * Firmware 1.9.4.2 has two ways to put a CAN engine on a pin pair, and only one
 * of them is real on a Nano.  bus_can_set_pin (0x08019A60) hardwires the two
 * standard pairs to dedicated transceiver/relay GPIOs: port 0 drives CAN1 on
 * pins 6/14 via can1_claim_pins_6_14, and port 1 with pin1==3 && pin2==11
 * drives the PD4 select relay plus the CAN2 transceiver.  Any *other* pin pair
 * falls through to routing_matrix_build (0x08028EA6), the arbitrary routing
 * matrix -- which computes a routing image into RAM at 0x2000CFD8 and then
 * calls routing_matrix_commit_stub (0x08028E28) to push it to the hardware.
 * That function is `bx lr`: a stub.  The matrix is a bank of I2C GPIO expanders
 * (probed at 0x40/0x42/0x44/0x48/0x4A by i2c_expander_probe, 0x0802B358) that a
 * Nano does not carry, so an arbitrary pin request is accepted, decoded,
 * written to a shadow buffer, and dropped.  The channel then reports success
 * and hears nothing, which is the worst way to fail -- so refuse those
 * instead.
 *
 * K-line/L-line have no escape hatch either.  uart_hw_set_pin (0x0801C890)
 * hardwires pin 7 via kline_claim_pin_7 (0x080291E4, the UART-side twin of
 * can1_claim_pins_6_14) and routes every other pin -- L-line/15 included --
 * through the same routing_matrix_build/commit-stub dead end.  So pin 7 is the
 * only K-line pin this hardware drives; do not add K-line/L-line pin-switching,
 * for the same reason the CAN whitelist below stops at 060E/030B.  Fuller
 * write-up (the five UART engines, the debug-menu-only exception) in
 * notes/can_bus_selection.md. */
#define VCX_PINS_HS_CAN 0x060Eu     /* pins 6/14  -> CAN1 */
#define VCX_PINS_MS_CAN 0x030Bu     /* pins 3/11  -> CAN2 */
#define VCX_PINS_KLINE  0x0700u     /* pin 7      -> the one wired K-line route */

/* Which CAN controller serves a pin pair, or false to leave the firmware's own
 * choice alone.  vcx_nano.ini overrides for bench work:
 *   can_bus=0 / can_bus=1   force that controller (and allow any pin pair)
 *   can_bus=off             send only the pins, let the firmware pick */
static bool can_bus_for_pins(uint32_t pins, uint32_t *bus, bool *forced)
{
    char v[16];
    *forced = false;
    if (ini_get("can_bus", v, sizeof(v))) {
        *forced = true;
        if (v[0] == 'o' || v[0] == 'O' || v[0] == 'n' || v[0] == 'N') return false;
        *bus = (uint32_t)atoi(v);
        return true;
    }
    *bus = (pins == 0 || pins == VCX_PINS_HS_CAN) ? 0u : 1u;
    return true;
}

/* PID_BUS_PIN for a UART/K-line engine.
 *
 * The firmware's UART engines take their pin from comparam 0x0002 alone:
 * ctor_iso9141_uart (0x08032F44) memsets the whole engine object, its
 * set_comparam (0x08032A1E) case 2 stores value byte 3 into obj+0xC24 (pin1)
 * and byte 2 into obj+0xC25 (pin2), and start() (0x080327B0) hands obj+0xC24
 * straight to uart_hw_set_pin.  There is no default-to-7 anywhere on that path,
 * so a channel that never receives PID_BUS_PIN starts with pin1 = 0 and
 * uart_hw_set_pin sends it to routing_matrix_build instead of
 * kline_claim_pin_7 -- the K-line GPIO/enable claim never happens.
 *
 * The vendor DLL does not rely on a default either.  Its ISO9141 comm-param
 * builder (VCXPT32.dll 0x10004FB0, used for both PROTOCOL_ISO9141 and
 * ISO9141_PS) writes PID_BUS_PIN pin1 = 7 literally for the base protocol and
 * pin1 = the J1962_PINS high byte for the _PS variant; the HONDA_DIAGH_PS
 * builder (0x10005840) always takes the J1962_PINS high byte.  pin2 is left
 * zero in every UART case, which is what uart_hw_set_pin's `pin1 == 7 &&
 * pin2 == 0` test wants.  So: always send it, pin1 in bits 31..24. */
#define VCX_UART_PIN_KLINE 7u

/* vcx_nano.ini escape hatch for the Phase 2 bench work on issue #7:
 *   kline_pin=N     drive UART pin N instead of 7
 *   kline_pin=any   let the application's J1962_PINS through unchecked
 * Both send a pin the routing matrix cannot reach, so they are research-only:
 * the channel will start clean and hear nothing. */
static bool kline_pin_override(uint8_t *pin, bool *any)
{
    char v[16];
    *any = false;
    if (!ini_get("kline_pin", v, sizeof(v))) return false;
    if (v[0] == 'a' || v[0] == 'A') { *any = true; return true; }
    *pin = (uint8_t)atoi(v);
    return true;
}

/* kline_pin=any only, without the pin value kline_pin_override also computes
 * -- callers that just need to know whether the bypass is on don't have a
 * pin to pass back. */
static bool kline_pin_any(void)
{
    uint8_t pin = 0; bool any = false;
    kline_pin_override(&pin, &any);
    return any;
}

static uint8_t uart_pin_default(void)
{
    uint8_t pin = VCX_UART_PIN_KLINE; bool any = false;
    if (kline_pin_override(&pin, &any) && !any) return pin;
    return VCX_UART_PIN_KLINE;
}

/* Rewrite the device filter table after a removal: re-send every cached record
 * at its (possibly shifted) host index, then delete the slots the shift left
 * behind.  FILTINIT with mode 1 only sets the filter mode -- pfilter_list_set
 * (0x08019E22) clears the slots for mode 2 alone -- so a vacated slot stays
 * live until a size-0 record (pfilter_list_add's delete, 0x08019E58) clears it.
 * prev_n is the record count before the removal. */
static long filters_replay(uint8_t chan, int prev_n)
{
    uint8_t one = 0x01;
    long st = vcx_xact(0x00, VCX_OP_FILTINIT, chan, &one, 1, NULL, NULL, 0);
    if (st < 0) return -1;
    if (st != VCX_STATUS_OK) return ERR_FAILED;
    for (int i = 0; i < s_filt_n[chan]; i++) {
        s_filt[chan][i][VCX_FILT_SLOT_BYTE] = (uint8_t)i;
        st = vcx_xact(0x00, VCX_OP_ADDFILT, chan, s_filt[chan][i],
                      (uint16_t)s_filt_len[chan][i], NULL, NULL, 0);
        if (st < 0) return -1;
        if (st != VCX_STATUS_OK) return ERR_FAILED;
    }
    for (int i = s_filt_n[chan]; i < prev_n; i++) {
        /* count=1, slot i, type 0, flag 0, size 0 */
        uint8_t del[5] = { 0x01, (uint8_t)i, 0x00, 0x00, 0x00 };
        st = vcx_xact(0x00, VCX_OP_ADDFILT, chan, del, sizeof(del), NULL, NULL, 0);
        if (st < 0) return -1;
        if (st != VCX_STATUS_OK) return ERR_FAILED;
    }
    return STATUS_NOERROR;
}

/* ---- PT_CMD_* translation ---- */

/* OPEN -> PARAMS -> FILTINIT -> START, in that order and always from a fresh
 * open.  The CAN driver latches its port, pins and bitrate when the channel
 * starts, and a channel that has already been started once will not pick up a
 * new port from a stop/start alone -- bus_can_init reuses the object it already
 * built.  Measured on the bench with tools/mscan_probe.py: parameters set
 * before the first start reach MS-CAN (114 frames), the same parameters pushed
 * via stop/params/start reach nothing.  So a bus change reopens the channel. */
static long chan_bring_up(uint8_t chan, uint16_t engine,
                          const uint8_t *params, int plen)
{
    uint8_t op[4] = {0x00, 0x00, (uint8_t)(engine>>8), (uint8_t)engine};
    long st = vcx_xact(0x00, VCX_OP_OPEN, chan, op, 4, NULL, NULL, 0);
    if (st == 0xFE) {
        STAT_BUMP(lic_open_gate_fe);
        if (license_unlock()) {
            st = vcx_xact(0x00, VCX_OP_OPEN, chan, op, 4, NULL, NULL, 0);
            if (st == VCX_STATUS_OK) STAT_BUMP(lic_open_recovered);
        }
    }
    if (st < 0) return -1;
    if (st != VCX_STATUS_OK) return (st == 0xFE) ? ERR_DEVICE_NOT_CONNECTED : ERR_FAILED;

    uint8_t one = 0x01, chb = chan;
    if (vcx_xact(0x00, VCX_OP_PARAMS,   chan, params, (uint16_t)plen,
                 NULL, NULL, 0) != VCX_STATUS_OK ||
        vcx_xact(0x00, VCX_OP_FILTINIT, chan, &one, 1, NULL, NULL, 0) != VCX_STATUS_OK ||
        vcx_xact(0x00, VCX_OP_START,    chan, &chb, 1, NULL, NULL, 0) != VCX_STATUS_OK) {
        /* OPEN went through but a later step did not.  Unwind it with the same
         * STOP+CLOSE do_disconnect uses, or the device keeps a channel it thinks
         * is open while the host holds no table entry for it -- the next connect
         * reuses this slot index and issues OPEN on an already-open channel.  A
         * single transient PARAMS/FILTINIT/START timeout is enough to reach here
         * now that a stray control-transaction timeout only fails its own call
         * instead of ending the session.  Best-effort: we have already failed
         * and return ERR_FAILED whatever these two report. */
        vcx_xact(0x00, VCX_OP_STOP,  chan, NULL, 0, NULL, NULL, 0);
        vcx_xact(0x00, VCX_OP_CLOSE, chan, NULL, 0, NULL, NULL, 0);
        return ERR_FAILED;
    }
    return STATUS_NOERROR;
}

static long do_connect(const uint8_t *p, uint8_t *resp, uint16_t *rl, uint16_t cap)
{
    uint32_t proto = (uint32_t)p[0] | (p[1]<<8) | (p[2]<<16) | ((uint32_t)p[3]<<24);
    uint32_t flags = (uint32_t)p[4] | (p[5]<<8) | (p[6]<<16) | ((uint32_t)p[7]<<24);
    uint32_t baud  = (uint32_t)p[8] | (p[9]<<8) | (p[10]<<16) | ((uint32_t)p[11]<<24);
    uint16_t engine = proto_to_engine(proto);
    if (!engine) return ERR_INVALID_PROTOCOL_ID;

    int chan = -1;
    EnterCriticalSection(&s_chan);
    /* Reserve the slot (in_use=true) before releasing the lock, closing the
     * window where a second concurrent do_connect() could pick this same
     * index -- dev_channel_add() later finalizes this same reserved slot by
     * wire_id instead of scanning for a free one. */
    for (int i = 0; i < DEV_MAX_CHANNELS; i++) if (!s_channels[i].in_use) { chan = i; s_channels[i].in_use = true; break; }
    LeaveCriticalSection(&s_chan);
    if (chan < 0) return ERR_EXCEEDED_LIMIT;

    /* A forced link loss may leave a tracked firmware timer behind. Never
     * reuse its channel until the stop can be confirmed on the device. */
    long cleanup = periodic_clear_chan((uint8_t)chan);
    if (cleanup != STATUS_NOERROR) {
        EnterCriticalSection(&s_chan);
        s_channels[chan].in_use = false;
        LeaveCriticalSection(&s_chan);
        return cleanup;
    }

    if (!license_is_fresh()) { STAT_BUMP(lic_refresh_run); (void)license_unlock(); }
    else STAT_BUMP(lic_refresh_skipped);

    uint8_t par[24]; int par_n = 0;
    par_emit(par, &par_n, sizeof(par), VCX_PID_BAUDRATE, baud);
    s_uart_pin[chan] = 0;
    if (engine_is_kline(engine)) {
        /* Claim the K-line pin explicitly -- the engine object's pin field is
         * zero out of its constructor, and pin 0 routes to the dead matrix
         * instead of kline_claim_pin_7.  See uart_pin_default(). */
        s_uart_pin[chan] = uart_pin_default();
        emit_kline_pin(par, &par_n, sizeof(par), s_uart_pin[chan]);
    }
    /* Map the connect Flags into supported comm-params (checksum) and cache
     * them so a later pin-change reopen replays them -- see the repin path. */
    s_connect_flags[chan] = flags;
    emit_connect_flag_params(par, &par_n, sizeof(par), engine, flags);
    if (flags & (J2534_CONNECT_ISO9141_K_LINE_ONLY | PT_CONNECT_LISTEN_ONLY |
                 PT_CONNECT_SELF_TEST))
        dev_log("  connect flags 0x%08lX: K_LINE_ONLY/LISTEN_ONLY/SELF_TEST have "
                "no comm-param on this device (decoded, not pushed)",
                (unsigned long)flags);
    long st = chan_bring_up((uint8_t)chan, engine, par, par_n);
    if (st != STATUS_NOERROR) {
        /* chan_bring_up has already unwound the device side (STOP+CLOSE) if OPEN
         * had gone through.  Clear this slot's host-side caches too: the failed
         * connect leaves no channel-table entry, so the next connect reuses this
         * index, and without this reset it would inherit a prior occupant's
         * config/pin/filter bookkeeping (do_disconnect clears s_filt_n, but
         * s_cfg_n and s_pins_set are only ever reset on the success path below). */
        s_cfg_n[chan] = 0;
        s_pins_set[chan] = false;
        s_filt_n[chan] = 0;
        /* Release the reservation made above -- this connect never reaches
         * dev_channel_add(), so nothing else will clear in_use for this slot. */
        EnterCriticalSection(&s_chan);
        s_channels[chan].in_use = false;
        LeaveCriticalSection(&s_chan);
        return st;
    }

    s_cfg_n[chan] = 0;
    s_pins_set[chan] = false;
    cfg_set(chan, J2534_CFG_DATA_RATE, baud);
    uint32_t wire_id = (uint32_t)chan + 1;
    if (resp && cap >= 4) {
        resp[0]=(uint8_t)wire_id; resp[1]=(uint8_t)(wire_id>>8);
        resp[2]=(uint8_t)(wire_id>>16); resp[3]=(uint8_t)(wire_id>>24);
        if (rl) *rl = 4;
    }
    return STATUS_NOERROR;
}

static long do_disconnect(const uint8_t *p)
{
    uint32_t wire_id = (uint32_t)p[0] | (p[1]<<8) | (p[2]<<16) | ((uint32_t)p[3]<<24);
    if (wire_id == 0) return ERR_INVALID_CHANNEL_ID;
    uint8_t chan = (uint8_t)(wire_id - 1);
    long cleanup = periodic_clear_chan(chan);
    if (cleanup != STATUS_NOERROR) return cleanup;
    vcx_xact(0x00, VCX_OP_STOP,  chan, NULL, 0, NULL, NULL, 0);
    long st = vcx_xact(0x00, VCX_OP_CLOSE, chan, NULL, 0, NULL, NULL, 0);
    if (st == VCX_STATUS_OK) dev_channel_remove(wire_id);
    return (st == VCX_STATUS_OK) ? STATUS_NOERROR : (st < 0 ? -1 : ERR_FAILED);
}

static long do_write(const uint8_t *p, uint16_t plen)
{
    /* p: ChannelID(4) Timeout(4) TxFlags(4) DataSize(2) Data[] */
    uint32_t wire_id = (uint32_t)p[0] | (p[1]<<8) | (p[2]<<16) | ((uint32_t)p[3]<<24);
    uint32_t txflags = (uint32_t)p[8] | (p[9]<<8) | (p[10]<<16) | ((uint32_t)p[11]<<24);
    uint16_t dsize   = (uint16_t)(p[12] | (p[13]<<8));
    if (wire_id == 0 || wire_id > DEV_MAX_CHANNELS) return ERR_INVALID_CHANNEL_ID;
    if (14 + (int)dsize > plen) return ERR_INVALID_MSG;
    uint8_t chan = (uint8_t)(wire_id - 1);
    const uint8_t *data = &p[14];
    uint8_t body[6 + 4128]; int n = 0;
    body[n++]=(uint8_t)(txflags>>24); body[n++]=(uint8_t)(txflags>>16);
    body[n++]=(uint8_t)(txflags>>8);  body[n++]=(uint8_t)txflags;
    body[n++]=(uint8_t)(dsize>>8);    body[n++]=(uint8_t)dsize;
    memcpy(&body[n], data, dsize); n += dsize;

    /* What the application expects to read back for its own transmit, in the
     * order J2534-1 04.04 puts it in the queue: loopback echo, then TxDone,
     * then whatever the ECU answers.  Without the TxDone, FORScan takes the
     * ECU response for the confirmation it was waiting on, drops it, and times
     * out on the now-empty queue -- exactly what the 2026-08-28 car log shows:
     * a good 62 02 00 01 from 7E8, then ERR_TIMEOUT, three times over.
     * TX_CAN_29BIT_ID and TX_ISO15765_ADDR_TYPE share bit positions with the
     * matching RxStatus bits, so they carry straight across. */
    uint16_t engine = proto_to_engine(chan_protocol(wire_id));

    /* A segmented ISO15765 transmit needs a flow-control filter so the engine
     * knows how to pace the multi-frame send against the ECU's flow control.
     * Standard single-frame tops out at DataSize 11 (4-byte id + 7 data); beyond
     * that, a missing FC filter is ERR_NO_FLOW_CONTROL.  Vendor-compatible
     * default only logs it (the firmware tolerated the send); strict returns the
     * spec's error.  s_filt is touched only on app threads, matching
     * filters_replay's lockless read. */
    if (engine_is_iso15765(engine) && dsize > 11) {
        bool have_fc = false;
        for (int i = 0; i < s_filt_n[chan]; i++)
            if (s_filt_len[chan][i] >= 4 &&
                s_filt[chan][i][2] == 0x02 && s_filt[chan][i][3] == 0x02) {
                have_fc = true; break;
            }
        if (!have_fc) {
            if (dev_strict_validation()) return ERR_NO_FLOW_CONTROL;
            dev_log("  tx ch=%u ISO15765 multi-frame (%u bytes) with no "
                    "flow-control filter (ERR_NO_FLOW_CONTROL under "
                    "strict_validation)", chan, dsize);
        }
    }

    uint32_t carry = txflags & (J2534_TX_CAN_29BIT_ID | J2534_TX_ISO15765_ADDR_TYPE);
    uint32_t loopback = cfg_get(chan, J2534_CFG_LOOPBACK);
    tx_confirm_t confirm[2]; int nconfirm = 0;
    if (loopback) {
        confirm[nconfirm].rx_status = J2534_RX_TX_MSG_TYPE | carry;
        confirm[nconfirm].data = data;
        confirm[nconfirm].len = dsize;
        nconfirm++;
    }
    if (engine_is_iso15765(engine) && dsize >= 4) {
        /* TxDone carries the transmitted CAN id and nothing else. */
        confirm[nconfirm].rx_status = J2534_RX_TX_MSG_TYPE | J2534_RX_TX_INDICATION | carry;
        confirm[nconfirm].data = data;
        confirm[nconfirm].len = 4;
        nconfirm++;
    }

    long rc = vcx_send_msg(chan, body, n, confirm, nconfirm);
    if (rc == STATUS_NOERROR && dev_log_enabled())
        dev_log("  tx ch=%u queued %d confirmation(s) (loopback=%lu engine=%04X)",
                chan, nconfirm, (unsigned long)loopback, engine);
    return rc;
}

static long do_filter(const uint8_t *p, uint16_t plen, uint8_t *resp, uint16_t *rl, uint16_t cap)
{
    /* p: ChannelID(4) FilterType(4) flags(4) mlen(2) plen(2) fclen(2) mask pattern fc */
    uint32_t wire_id = (uint32_t)p[0] | (p[1]<<8) | (p[2]<<16) | ((uint32_t)p[3]<<24);
    uint32_t ftype   = (uint32_t)p[4] | (p[5]<<8) | (p[6]<<16) | ((uint32_t)p[7]<<24);
    uint16_t mlen = (uint16_t)(p[12] | (p[13]<<8));
    uint16_t pl   = (uint16_t)(p[14] | (p[15]<<8));
    uint16_t fcl  = (uint16_t)(p[16] | (p[17]<<8));
    if (wire_id == 0 || wire_id > DEV_MAX_CHANNELS) return ERR_INVALID_CHANNEL_ID;
    if (18 + mlen + pl + fcl > plen) return ERR_INVALID_MSG;
    const uint8_t *mask = &p[18], *patt = mask + mlen, *fc = patt + pl;
    uint8_t chan = (uint8_t)(wire_id - 1);

    uint32_t proto = chan_protocol(wire_id);
    uint32_t flags = rd32le(p + 8);   /* OR of the three filter msgs' TxFlags */

    /* Centralized type/length/protocol validation: bad type or flow control on a
     * non-CAN protocol -> ERR_NOT_SUPPORTED; out-of-range widths -> ERR_INVALID_MSG. */
    long vst = pt_validate_filter(proto, ftype, mlen, pl, fcl,
                                  dev_strict_validation());
    if (vst != STATUS_NOERROR) return vst;

    /* BLOCK is matched host-side: the firmware command-0x48 block record was
     * never captured, so the DLL stores the rule and drops matching frames in
     * route_rx.  Ids come from the shared s_filter_seq space so StopMsgFilter
     * and CLEAR_MSG_FILTERS reach block rules too. */
    if (ftype == J2534_BLOCK_FILTER) {
        EnterCriticalSection(&s_chan);
        for (int i = 0; i < VCX_MAX_BLOCK; i++) {
            block_rule_t *b = &s_block[chan][i];
            if (b->in_use && b->len == mlen &&
                memcmp(b->mask, mask, mlen) == 0 &&
                memcmp(b->patt, patt, mlen) == 0) {
                LeaveCriticalSection(&s_chan);
                return ERR_NOT_UNIQUE;
            }
        }
        int bslot = -1;
        for (int i = 0; i < VCX_MAX_BLOCK; i++)
            if (!s_block[chan][i].in_use) { bslot = i; break; }
        if (bslot < 0) { LeaveCriticalSection(&s_chan); return ERR_EXCEEDED_LIMIT; }
        uint32_t bid = ++s_filter_seq[chan];
        block_rule_t *b = &s_block[chan][bslot];
        b->in_use = true; b->id = bid; b->len = (uint8_t)mlen;
        memcpy(b->mask, mask, mlen); memcpy(b->patt, patt, mlen);
        LeaveCriticalSection(&s_chan);
        if (resp && cap >= 4) { wr32le(resp, bid); if (rl) *rl = 4; }
        return STATUS_NOERROR;
    }

    uint8_t f[VCX_FILTER_MAXLEN]; int n = 0;
    /* Firmware command 0x48 reads byte 0 as the number of encoded filters,
     * not the J2534 FilterID.  Filter IDs are host-side bookkeeping. */
    f[n++] = 0x01;
    if (ftype == J2534_PASS_FILTER) {
        /* Command 0x48 is protocol-neutral: its record supplies an identifier
         * length followed by one pattern and one mask of that exact length.
         * The J1850/UART engines use the same vtable slots and command wrapper,
         * so retain their native 1..12-byte width instead of forcing CAN's four.
         * Pattern first, like the flow-control record: the firmware stores the
         * first array as list_msg (the match value) and the second as mask_msg,
         * and VCXPT32 copies pattern then mask for every filter type.  The only
         * captured PASS record was pass-all 0/0, which reads the same either way
         * (notes/odis_jetta_2014_pretest_review.md F5 on odis_jetta_2014_testing). */
        f[n++]=0x00 /* slot, set below */; f[n++]=0x01; f[n++]=0x01; f[n++]=(uint8_t)mlen;
        memcpy(&f[n], patt, pl); n += pl;
        memcpy(&f[n], mask, mlen); n += mlen;
    } else {   /* FLOW_CONTROL (validator has confirmed CAN + widths >= 4) */
        /* Width 4 -- the CAN id -- is the only form of this record the capture
         * ever proved.  A width-5 variant (id plus the extended-addressing
         * address byte) was tried here, keyed off ISO15765_ADDR_TYPE, but the
         * only flag word this layer has is the OR of the mask, pattern and
         * flow-control messages' TxFlags (api.c packs them together), so the
         * address byte on any ONE of the three would have switched the whole
         * record to a wire format no capture backs.  A misparsed record makes
         * the filter silently stop matching and every multi-frame response for
         * that id disappear, which is far worse than not carrying the address
         * byte, so stay on the captured width until a width-5 record is
         * observed on the wire. */
        const uint16_t w = 4;
        if (flags & J2534_TX_ISO15765_ADDR_TYPE)
            dev_log("  filter ch=%u extended addressing requested; installing the "
                    "id-only (4-byte) flow-control record, the only captured form",
                    chan);
        f[n++]=0x00 /* slot, set below */; f[n++]=0x02; f[n++]=0x02; f[n++]=(uint8_t)w;
        memcpy(&f[n], patt, w); n+=w; memcpy(&f[n], mask, w); n+=w; memcpy(&f[n], fc, w); n+=w;
    }
    if (s_filt_n[chan] >= VCX_MAX_FILTERS) return ERR_EXCEEDED_LIMIT;
    /* Each filter gets its own firmware slot.  Every record used to go out as
     * slot 0, so each new filter overwrote the previous one and only the most
     * recently added flow-control id was ever answered (FORScan: the GEM went
     * deaf once the IC's filter was added after it). */
    f[VCX_FILT_SLOT_BYTE] = (uint8_t)s_filt_n[chan];
    /* Uniqueness: an identical record, apart from its slot, is already installed. */
    for (int i = 0; i < s_filt_n[chan]; i++)
        if (s_filt_len[chan][i] == n && s_filt[chan][i][0] == f[0] &&
            memcmp(s_filt[chan][i] + VCX_FILT_SLOT_BYTE + 1, f + VCX_FILT_SLOT_BYTE + 1,
                   n - VCX_FILT_SLOT_BYTE - 1) == 0)
            return ERR_NOT_UNIQUE;
    uint32_t fid = ++s_filter_seq[chan];
    long st = vcx_xact(0x00, VCX_OP_ADDFILT, chan, f, n, NULL, NULL, 0);
    if (st < 0) { s_filter_seq[chan]--; return -1; }
    if (st != VCX_STATUS_OK) { s_filter_seq[chan]--; return ERR_FAILED; }
    int slot = s_filt_n[chan]++;
    memcpy(s_filt[chan][slot], f, n);
    s_filt_len[chan][slot] = n;
    s_filt_id[chan][slot] = fid;
    if (resp && cap >= 4) { wr32le(resp, fid); if (rl) *rl = 4; }
    return STATUS_NOERROR;
}

static long do_stop_filter(const uint8_t *p)
{
    uint32_t wire_id = rd32le(p), fid = rd32le(p + 4);
    if (wire_id == 0 || wire_id > DEV_MAX_CHANNELS || !dev_channel_find(wire_id))
        return ERR_INVALID_CHANNEL_ID;
    uint8_t chan = (uint8_t)(wire_id - 1);
    int at = -1;
    for (int i = 0; i < s_filt_n[chan]; i++)
        if (s_filt_id[chan][i] == fid) { at = i; break; }
    if (at < 0) {
        /* Not a device filter -- it may be a host-side BLOCK rule, which shares
         * the id space.  Clearing one needs no filter-bank replay. */
        EnterCriticalSection(&s_chan);
        for (int i = 0; i < VCX_MAX_BLOCK; i++)
            if (s_block[chan][i].in_use && s_block[chan][i].id == fid) {
                s_block[chan][i].in_use = false;
                LeaveCriticalSection(&s_chan);
                return STATUS_NOERROR;
            }
        LeaveCriticalSection(&s_chan);
        return ERR_INVALID_FILTER_ID;
    }
    for (int i = at + 1; i < s_filt_n[chan]; i++) {
        memcpy(s_filt[chan][i - 1], s_filt[chan][i], VCX_FILTER_MAXLEN);
        s_filt_len[chan][i - 1] = s_filt_len[chan][i];
        s_filt_id[chan][i - 1] = s_filt_id[chan][i];
    }
    s_filt_n[chan]--;
    return filters_replay(chan, s_filt_n[chan] + 1);
}

static long do_clear_filters(uint32_t wire_id)
{
    if (wire_id == 0 || wire_id > DEV_MAX_CHANNELS || !dev_channel_find(wire_id))
        return ERR_INVALID_CHANNEL_ID;
    uint8_t chan = (uint8_t)(wire_id - 1);
    EnterCriticalSection(&s_chan);
    memset(s_block[chan], 0, sizeof(s_block[chan]));
    LeaveCriticalSection(&s_chan);
    int prev_n = s_filt_n[chan];
    s_filt_n[chan] = 0;
    return filters_replay(chan, prev_n);
}

/* Firmware-native periodic timers, recovered on 2026-08-30
 * (notes/periodic_timer_vendor_capture_2026-08-30.md).  The op-0x4A add record
 * is:  01 <slot> C9 <buflen> 00 <interval_us LE32> <buffer[buflen]>  where the
 * buffer is four zero TX-flag bytes followed by the J2534 message Data (the 4-byte
 * CAN id big-endian plus payload), so buflen = 4 + DataSize.  interval_us is the
 * J2534 milliseconds times 1000.  Op 0x49 value 01 enables the service (the
 * first frame goes out immediately, matching J2534); a zero record stops one
 * slot; op 0x49 value 02 clears every slot on the channel.
 *
 * Used for the two engines the capture actually proved -- CAN (0x8101) and
 * ISO15765 (0x8001).  Everything else (K-line/J1850, the J1939 and TP2.0
 * engines that engine_is_can() also matches, a message carrying TxFlags this
 * record has no field for, or the ini override) falls back to the host
 * scheduler below.  Moving the proven cases onto the device removes the host
 * scheduler's ~1 ms jitter and its dependence on Windows timer resolution.
 *
 * Record flag 0x80 makes ptimer_list_tx decode buffer[0..3] as big-endian
 * PDU TX flags, NOT a CAN id. Both CAN engines read the id from Data[0..3].
 * See notes/filter_periodic_recovery.md section 9 for the instruction trace.
 * Keep the captured zero-TX-flags scope: nonzero flags (29-bit ids, extended
 * addressing, frame padding) still use the host scheduler until separately
 * validated for firmware timers. The record flag byte 0xC9 is distinct from
 * these four PDU TX-flag bytes. */
#define VCX_FW_PERIODIC_TXFLAGS_OK 0u   /* only zero TX flags validated here */

static bool fw_periodic_eligible(uint16_t engine, uint32_t txflags, uint16_t len)
{
    if (engine != 0x8001 && engine != 0x8101) return false;
    if (txflags & ~VCX_FW_PERIODIC_TXFLAGS_OK) return false;
    return (4u + (uint32_t)len) <= 16u;
}

static long fw_periodic_add(uint8_t chan, uint8_t slot, uint32_t interval_ms,
                            const uint8_t *data, uint16_t dsize)
{
    uint8_t f[5 + 4 + 4 + 12]; int n = 0;
    uint32_t us = interval_ms * 1000u;
    f[n++] = 0x01;              /* one record */
    f[n++] = slot;
    f[n++] = 0xC9;              /* flags, per capture */
    f[n++] = (uint8_t)(4 + dsize);   /* buffer length: TX flags + Data */
    f[n++] = 0x00;              /* match length */
    f[n++] = (uint8_t)us; f[n++] = (uint8_t)(us >> 8);
    f[n++] = (uint8_t)(us >> 16); f[n++] = (uint8_t)(us >> 24);
    f[n++] = 0; f[n++] = 0; f[n++] = 0; f[n++] = 0;   /* BE32 PDU TX flags */
    memcpy(&f[n], data, dsize); n += dsize;
    long st = vcx_xact(0x00, VCX_OP_PERIODIC_ADD, chan, f, (uint16_t)n, NULL, NULL, 0);
    if (st < 0) return -1;
    if (st != VCX_STATUS_OK) return ERR_FAILED;
    uint8_t en = 0x01;
    st = vcx_xact(0x00, VCX_OP_PERIODIC_CTRL, chan, &en, 1, NULL, NULL, 0);
    if (st < 0) return -1;
    if (st != VCX_STATUS_OK) return ERR_FAILED;
    return STATUS_NOERROR;
}

/* Stop one firmware timer slot with the zero record from the capture. */
static long fw_periodic_stop(uint8_t chan, uint8_t slot)
{
    uint8_t f[9] = { 0x01, slot, 0, 0, 0, 0, 0, 0, 0 };
    long st = vcx_xact(0x00, VCX_OP_PERIODIC_ADD, chan, f, sizeof(f), NULL, NULL, 0);
    return st == VCX_STATUS_OK ? STATUS_NOERROR : st < 0 ? -1 : ERR_FAILED;
}

/* Five-baud (slow) init support. Power-on slow-init defaults per engine (us),
 * from each engine's *_init_defaults (notes/uart_slow_init_firmware.md). */
typedef struct {
    uint16_t engine;
    uint32_t tidle, w1, w2, w3, w4_min, w4_max;
} slow_defaults_t;
static const slow_defaults_t s_slow_defaults[] = {
    {0x9104,  300000u, 300000u,  20000u,  20000u, 25000u, 300000u},  /* ISO9141 */
    {0x9004,  300000u, 300000u,  20000u,  20000u, 25000u, 300000u},  /* ISO14230 */
    {0x9B04, 2000000u, 300000u,  20000u,  20000u, 25000u, 300000u},  /* KW82 */
    {0x9204,  300000u, 500000u, 100000u, 100000u, 25000u,  50000u},  /* KW1281 */
};
static uint32_t s_five_baud_restore_us[DEV_MAX_CHANNELS];  /* 0 = nothing to restore */

static bool cfg_has(int ch, uint32_t param)
{
    for (int i = 0; i < s_cfg_n[ch]; i++) if (s_cfg[ch][i].param == param) return true;
    return false;
}

/* The engine's effective window in us: the J2534 value (ms) when the app set
 * one -- the larger of W<n> and UART_ECHO_BYTE_PS's T<n> when both apply, since
 * the firmware keeps whichever arrived last -- else the power-on default. */
static uint32_t slow_window_us(int ch, uint32_t w, uint32_t ueb, uint32_t fw_default)
{
    bool has_w = cfg_has(ch, w), has_ueb = ueb && cfg_has(ch, ueb);
    if (!has_w && !has_ueb) return fw_default;
    uint32_t a = has_w ? cfg_get(ch, w) : 0, b = has_ueb ? cfg_get(ch, ueb) : 0;
    return (a > b ? a : b) * 1000u;
}

static long push_param(int ch, uint16_t pid, uint32_t value)
{
    uint8_t blob[6]; int bn = 0;
    par_emit(blob, &bn, sizeof(blob), pid, value);
    long st = vcx_xact(0x00, VCX_OP_PARAMS, (uint8_t)ch, blob, bn, NULL, NULL, 0);
    return st < 0 ? -1 : st != VCX_STATUS_OK ? ERR_FAILED : STATUS_NOERROR;
}

/* The firmware waits one idle comparam (0x45) before either init, so W5 (or
 * UART_ECHO_BYTE_PS's T0) cannot live there alongside TIDLE. Apply it for this
 * init only, and return the firmware's worst case until it pushes its record
 * or gives up: idle + 10 bits at 5 baud + W1 + W2 + W3 + W4min + W4max + 10 ms
 * (uart_slow_init_tx_tick / _sync_edge). Returns -1 on transport failure. */
long dev_five_baud_begin(uint32_t wire_id, uint32_t *worst_ms)
{
    *worst_ms = 0;
    if (wire_id == 0 || wire_id > DEV_MAX_CHANNELS) return ERR_INVALID_CHANNEL_ID;
    int ch = (int)wire_id - 1;
    uint16_t engine = proto_to_engine(chan_protocol(wire_id));
    const slow_defaults_t *d = NULL;
    for (size_t i = 0; i < sizeof(s_slow_defaults) / sizeof(s_slow_defaults[0]); i++)
        if (s_slow_defaults[i].engine == engine) d = &s_slow_defaults[i];
    if (!d) return STATUS_NOERROR;
    bool ueb = engine == 0x9204;
    uint32_t idle_now = cfg_has(ch, J2534_CFG_TIDLE) ? cfg_get(ch, J2534_CFG_TIDLE) * 1000u
                                                     : d->tidle;
    uint32_t idle_param = ueb ? J2534_2_CFG_UEB_T0_MIN : J2534_CFG_W5;
    uint32_t idle = cfg_has(ch, idle_param) ? cfg_get(ch, idle_param) * 1000u : idle_now;
    s_five_baud_restore_us[ch] = 0;
    if (idle != idle_now) {
        EnterCriticalSection(&s_link);
        long st = push_param(ch, VCX_PID_ISO_TIDLE, idle);
        LeaveCriticalSection(&s_link);
        if (st != STATUS_NOERROR) return st;
        s_five_baud_restore_us[ch] = idle_now;
        dev_log("  five-baud idle %lu us for this init (fast-init idle %lu us restored after)",
                (unsigned long)idle, (unsigned long)idle_now);
    }
    uint64_t us = (uint64_t)idle + 2000000u
        + slow_window_us(ch, J2534_CFG_W1, ueb ? J2534_2_CFG_UEB_T1_MAX : 0, d->w1)
        + slow_window_us(ch, J2534_CFG_W2, ueb ? J2534_2_CFG_UEB_T2_MAX : 0, d->w2)
        + slow_window_us(ch, J2534_CFG_W3, ueb ? J2534_2_CFG_UEB_T3_MAX : 0, d->w3)
        + slow_window_us(ch, J2534_CFG_W4, 0, d->w4_min) + d->w4_max + 10000u;
    *worst_ms = (uint32_t)((us + 999u) / 1000u);
    return STATUS_NOERROR;
}

/* After the init: put the fast-init idle back. After a failure, re-enable every
 * channel's firmware periodic service: a W1 timeout after exactly one sync edge
 * leaves every firmware timer paused (uart_slow_init_sync_edge), which would
 * silently stop keep-alives running on other channels. Op 0x49 value 01 resumes
 * each live timer, and sends its message once immediately. */
void dev_five_baud_end(uint32_t wire_id, bool failed)
{
    if (wire_id == 0 || wire_id > DEV_MAX_CHANNELS) return;
    int ch = (int)wire_id - 1;
    EnterCriticalSection(&s_link);
    if (s_five_baud_restore_us[ch]) {
        if (push_param(ch, VCX_PID_ISO_TIDLE, s_five_baud_restore_us[ch]) != STATUS_NOERROR)
            dev_log("  five-baud: restoring the fast-init idle failed");
        s_five_baud_restore_us[ch] = 0;
    }
    if (failed) {
        uint8_t en = 0x01;
        for (int c = 0; c < DEV_MAX_CHANNELS; c++) {
            if (!s_fw_slot_used[c]) continue;
            long st = vcx_xact(0x00, VCX_OP_PERIODIC_CTRL, (uint8_t)c, &en, 1, NULL, NULL, 0);
            dev_log("  five-baud failed: re-enabled firmware periodics on channel %d (status %ld)",
                    c, st);
        }
    }
    LeaveCriticalSection(&s_link);
}

/* The firmware's value in us for a K-line timing param: the app's setting as
 * it was pushed, else the engine's power-on default. */
static uint64_t kline_timing_us(int ch, uint32_t param, uint32_t fw_default_us)
{
    return cfg_has(ch, param) ? j2534_to_vcx_value(param, cfg_get(ch, param)) : fw_default_us;
}

/* Derive the FAST_INIT wait from idle, wake-up, PDU/echo and end-of-frame
 * timing, plus a reply window and host slack. Unset values use engine defaults.
 * min_reply_ms excludes frames arriving before the wake-up pattern could end.
 * Calls require dev_api_lock. */
#define FAST_INIT_REPLY_WINDOW_MS 300u
#define FAST_INIT_HOST_SLACK_MS    50u
#define FAST_INIT_FLOOR_MS        500u
#define FAST_INIT_TICK_SLACK_MS    32u
long dev_fast_init_window(uint32_t wire_id, uint16_t pdu_len,
                          uint32_t *timeout_ms, uint32_t *min_reply_ms)
{
    *timeout_ms = FAST_INIT_FLOOR_MS;
    *min_reply_ms = 0;
    if (wire_id == 0 || wire_id > DEV_MAX_CHANNELS) return ERR_INVALID_CHANNEL_ID;
    int ch = (int)wire_id - 1;
    uint64_t p1 = kline_timing_us(ch, J2534_CFG_P1_MAX, 20000u);
    uint64_t p3 = kline_timing_us(ch, J2534_CFG_P3_MIN, 55000u);
    uint64_t p4 = kline_timing_us(ch, J2534_CFG_P4_MIN, 2000u);
    uint64_t idle = kline_timing_us(ch, J2534_CFG_TIDLE, 300000u);
    uint64_t twup = kline_timing_us(ch, J2534_CFG_TWUP, 50000u);
    uint32_t baud = cfg_get(ch, J2534_CFG_DATA_RATE);
    uint64_t byte_us = 10000000u / (baud >= 1200u ? baud : 1200u);  /* 10 bits */
    uint64_t pre = p1 > p3 ? p1 : p3;
    if (idle > pre) pre = idle;
    /* Preserve the reply allowance unless the configured P2_MAX needs more. */
    uint64_t reply_us = (uint64_t)FAST_INIT_REPLY_WINDOW_MS * 1000u;
    uint64_t p2 = kline_timing_us(ch, J2534_CFG_P2_MAX, 0);
    if (p2 > reply_us) reply_us = p2;
    uint64_t us = pre + twup + (uint64_t)pdu_len * (byte_us + p4) + 2u * (p1 + byte_us)
                + reply_us + (uint64_t)FAST_INIT_HOST_SLACK_MS * 1000u;
    uint64_t ms = (us + 999u) / 1000u;
    if (ms < FAST_INIT_FLOOR_MS) ms = FAST_INIT_FLOOR_MS;
    if (ms > 0xFFFFFFFFu) ms = 0xFFFFFFFFu;
    *timeout_ms = s_fast_init_timeout_ms ? s_fast_init_timeout_ms : (uint32_t)ms;
    uint64_t twup_ms = twup / 1000u;
    *min_reply_ms = twup_ms > FAST_INIT_TICK_SLACK_MS
                  ? (uint32_t)(twup_ms - FAST_INIT_TICK_SLACK_MS) : 0;
    return STATUS_NOERROR;
}

/* vcx_nano.ini: periodic=host forces every periodic onto the host scheduler
 * (bench/debug); default auto uses firmware timers for CAN-family channels. */
static bool periodic_force_host(void)
{
    char v[16];
    return ini_get("periodic", v, sizeof(v)) && (v[0] == 'h' || v[0] == 'H');
}

static void repeat_tick(DWORD now)
{
    /* Same order as the transport: s_io -> s_chan. Hold both through the
     * final write, so STOP/RX completion cannot leave a detached pending send.
     * This path never waits for a device reply or takes s_link. */
    EnterCriticalSection(&s_io);
    EnterCriticalSection(&s_chan);
    for (unsigned ch = 0; ch < DEV_MAX_CHANNELS; ++ch) {
        repeat_t *r = &s_repeat[ch];
        dev_channel_t *c = chan_find_locked(ch + 1);
        if (!r->used || !r->active) continue;
        if (!c || c->generation != r->generation || s_com == INVALID_HANDLE_VALUE ||
            InterlockedCompareExchange(&s_link_dead, 0, 0)) {
            r->active = false; r->error = ERR_DEVICE_NOT_CONNECTED;
            repeat_set_awaiting_locked((uint8_t)ch, false); continue;
        }
        if (r->awaiting) {
            /* Never talk over the ECU: resend only after a silent window.  Free
             * the wire for one scheduler pass first, so back-to-back resends to
             * a silent ECU cannot starve a held periodic message. */
            if (now - r->sent_at < s_repeat_reply_timeout_ms) continue;
            repeat_set_awaiting_locked((uint8_t)ch, false);
            r->no_reply = true; r->next = now; continue;
        }
        if ((int32_t)(now - r->next) < 0) continue;
        bool resend = r->no_reply;
        r->no_reply = false;
        uint8_t req[14 + 4128];
        wr32le(req, ch + 1); wr32le(req + 4, 0); wr32le(req + 8, r->flags);
        req[12] = (uint8_t)r->tx_len; req[13] = (uint8_t)(r->tx_len >> 8);
        memcpy(req + 14, r->tx, r->tx_len);
#ifdef VCX_REPEAT_TEST
        long rc = repeat_test_send(req, (uint16_t)(14 + r->tx_len));
#else
        long rc = do_write(req, (uint16_t)(14 + r->tx_len));
#endif
        DWORD waited = now - r->sent_at;
        bool first = !r->sent;
        r->sent = rc == STATUS_NOERROR;
        r->sent_at = now;
        r->next = now + r->interval; /* no catch-up burst after a delay */
        repeat_set_awaiting_locked((uint8_t)ch, r->sent);
        if (rc != STATUS_NOERROR) {
            r->active = false; r->error = rc < 0 ? ERR_DEVICE_NOT_CONNECTED : rc;
        }
        if (resend) {
            LONG n = STAT_BUMP(repeat_no_reply);
            if (stat_verbose(n))
                dev_log("repeat id=%lu ch=%u no reply in %lu ms, resending (#%ld)",
                        (unsigned long)r->id, ch + 1, (unsigned long)waited, n);
        }
        if (first)
            dev_log("repeat id=%lu ch=%u send rc=%ld active=%d interval=%lu p3=%lu reply_timeout=%lu",
                    (unsigned long)r->id, ch + 1, rc, r->active, (unsigned long)r->interval,
                    (unsigned long)r->p3_ms, (unsigned long)s_repeat_reply_timeout_ms);
        else
            dev_log("repeat id=%lu ch=%u send rc=%ld active=%d since_last=%lu ms",
                    (unsigned long)r->id, ch + 1, rc, r->active, (unsigned long)waited);
    }
    LeaveCriticalSection(&s_chan);
    LeaveCriticalSection(&s_io);
}

long dev_repeat_start(uint32_t channel, uint32_t interval, uint32_t condition,
                      uint32_t flags, const uint8_t *tx, uint16_t tx_len,
                      const uint8_t *mask, const uint8_t *pattern, uint16_t filter_len,
                      uint32_t *id)
{
    if (!channel || channel > DEV_MAX_CHANNELS) return ERR_INVALID_CHANNEL_ID;
    EnterCriticalSection(&s_chan);
    dev_channel_t *c = chan_find_locked(channel);
    repeat_t *r = &s_repeat[channel - 1];
    long rc = STATUS_NOERROR;
    if (!c) rc = ERR_INVALID_CHANNEL_ID;
    else if (c->protocol != J2534_ISO9141) rc = ERR_NOT_SUPPORTED;
    else if (s_connect_flags[channel - 1] & PT_CONNECT_LISTEN_ONLY) rc = ERR_NOT_SUPPORTED;
    else if (r->used) rc = ERR_EXCEEDED_LIMIT;
    else if (!tx || !mask || !pattern || !id || !tx_len || tx_len > sizeof(r->tx) ||
             !filter_len || filter_len > sizeof(r->mask) || interval < 5 || interval > 65535 ||
             condition > 1 || (flags & ~0x200u)) rc = ERR_INVALID_IOCTL_VALUE;
    else if (s_repeat_seq == UINT32_MAX) rc = ERR_EXCEEDED_LIMIT;
    if (rc == STATUS_NOERROR) {
        memset(r, 0, sizeof(*r));
        r->used = true; r->id = ++s_repeat_seq;
        r->generation = c->generation; r->interval = interval;
        r->condition = condition; r->flags = flags;
        r->tx_len = tx_len; r->filter_len = filter_len;
        memcpy(r->tx, tx, tx_len); memcpy(r->mask, mask, filter_len); memcpy(r->pattern, pattern, filter_len);
        /* HDS sets P3_MIN just before START and restores it after STOP. */
        r->p3_ms = cfg_get((int)channel - 1, J2534_CFG_P3_MIN);
        /* App-owned checksum (ISO9141_NO_CHECKSUM) on a self-consistent Honda
         * frame: only replies framed the same way may decide completion. */
        r->honda_framed = (flags & 0x200u) && honda_frame_valid(tx, tx_len);
        repeat_set_awaiting_locked((uint8_t)(channel - 1), false);
        r->quiet_until = repeat_clock();
        r->next = r->quiet_until + 10; /* first scheduler opportunity; not a full interval */
        *id = r->id;
    }
    LeaveCriticalSection(&s_chan);
    if (rc != STATUS_NOERROR) return rc;
#ifdef VCX_REPEAT_TEST
    if (repeat_test_manual) return STATUS_NOERROR;
#endif
    if (!s_periodic_thread) {
        if (!pin_worker_module()) { dev_repeat_stop(channel, *id); return ERR_FAILED; }
        InterlockedExchange(&s_periodic_run, 1);
        s_periodic_thread = CreateThread(NULL, 0, periodic_proc, NULL, 0, NULL);
        if (!s_periodic_thread) {
            InterlockedExchange(&s_periodic_run, 0);
            dev_repeat_stop(channel, *id); return ERR_FAILED;
        }
    }
    return STATUS_NOERROR;
}

long dev_repeat_query(uint32_t channel, uint32_t id, uint32_t *active)
{
    if (!channel || channel > DEV_MAX_CHANNELS) return ERR_INVALID_CHANNEL_ID;
    EnterCriticalSection(&s_chan);
    repeat_t *r = &s_repeat[channel - 1];
    dev_channel_t *c = chan_find_locked(channel);
    long rc = !c ? ERR_INVALID_CHANNEL_ID :
              !r->used || r->id != id || r->generation != c->generation ? ERR_INVALID_MSG_ID : r->error;
    if (rc == STATUS_NOERROR) *active = r->active ? 1 : 0;
    LeaveCriticalSection(&s_chan);
    return rc;
}

void dev_repeat_activate(uint32_t channel)
{
    EnterCriticalSection(&s_chan);
    s_repeat[channel - 1].quiet_until = repeat_clock();
    s_repeat[channel - 1].next = s_repeat[channel - 1].quiet_until + 10;
    s_repeat[channel - 1].active = true;
    repeat_set_awaiting_locked((uint8_t)(channel - 1), false);
    LeaveCriticalSection(&s_chan);
}

long dev_repeat_stop(uint32_t channel, uint32_t id)
{
    if (!channel || channel > DEV_MAX_CHANNELS) return ERR_INVALID_CHANNEL_ID;
    EnterCriticalSection(&s_io);
    EnterCriticalSection(&s_chan);
    repeat_t *r = &s_repeat[channel - 1];
    long rc = !r->used || r->id != id ? ERR_INVALID_MSG_ID : STATUS_NOERROR;
    if (rc == STATUS_NOERROR) {
        memset(r, 0, sizeof(*r));
        repeat_set_awaiting_locked((uint8_t)(channel - 1), false);
    }
    LeaveCriticalSection(&s_chan);
    LeaveCriticalSection(&s_io);
    return rc;
}

static DWORD WINAPI periodic_proc(LPVOID arg)
{
    (void)arg;
    while (InterlockedCompareExchange(&s_periodic_run, 1, 1)) {
        DWORD now = GetTickCount();
        /* While a repeat owns the wire (reply outstanding, or inside P3_MIN
         * after the ECU's last frame) hold the periodic -- not dropped, it goes
         * out once the turn is over. Snapshot first: never nest s_chan in s_per. */
        bool busy[DEV_MAX_CHANNELS];
        DWORD rnow = repeat_clock();
        EnterCriticalSection(&s_chan);
        for (unsigned ch = 0; ch < DEV_MAX_CHANNELS; ++ch) {
            repeat_t *r = &s_repeat[ch];
            busy[ch] = r->used && r->active &&
                       (r->awaiting || (int32_t)(rnow - r->quiet_until) < 0);
        }
        LeaveCriticalSection(&s_chan);
        for (int i = 0; i < DEV_MAX_PERIODIC; i++) {
            uint8_t req[26]; int req_len = 0; bool due = false; uint8_t chan = 0;
            EnterCriticalSection(&s_per);
            periodic_t *entry = &s_periodic[i];
            if (entry->in_use && !entry->fw && (int32_t)(now - entry->next) >= 0 &&
                !busy[entry->chan]) {
                chan = (uint8_t)entry->chan;
                wr32le(req, (uint32_t)entry->chan + 1);
                wr32le(req + 4, 0);
                wr32le(req + 8, entry->txflags);
                req[12] = (uint8_t)entry->len; req[13] = (uint8_t)(entry->len >> 8);
                memcpy(req + 14, entry->data, entry->len);
                req_len = 14 + entry->len;
                entry->next = now + entry->interval;
                due = true;
            }
            LeaveCriticalSection(&s_per);
            if (!due) continue;
            /* The ECU answers this one too: an active repeat waits out that
             * turn (reply or silent window, then P3_MIN) like its own. */
            EnterCriticalSection(&s_chan);
            repeat_t *r = &s_repeat[chan];
            if (r->used && r->active) {
                r->sent_at = rnow;
                repeat_set_awaiting_locked(chan, true);
                busy[chan] = true;
            }
            LeaveCriticalSection(&s_chan);
            (void)do_write(req, (uint16_t)req_len);
        }
        repeat_tick(repeat_clock());
        Sleep(1);
    }
    return 0;
}

/* Called with s_link held, excluding concurrent timer API mutations. The
 * scheduler reads under s_per; never hold s_per across a control transaction. */
static long periodic_stop_entry(int index)
{
    EnterCriticalSection(&s_per);
    periodic_t entry = s_periodic[index];
    LeaveCriticalSection(&s_per);
    if (!entry.in_use) return STATUS_NOERROR;
    if (entry.fw) {
        long rc = fw_periodic_stop(entry.chan, entry.fw_slot);
        if (rc != STATUS_NOERROR) return rc;
    }
    EnterCriticalSection(&s_per);
    if (entry.fw) s_fw_slot_used[entry.chan] &= (uint16_t)~(1u << entry.fw_slot);
    memset(&s_periodic[index], 0, sizeof(s_periodic[index]));
    LeaveCriticalSection(&s_per);
    return STATUS_NOERROR;
}

static long periodic_stop_all(void)
{
    EnterCriticalSection(&s_chan);
    memset(s_repeat, 0, sizeof(s_repeat));
    for (unsigned ch = 0; ch < DEV_MAX_CHANNELS; ++ch) repeat_set_awaiting_locked((uint8_t)ch, false);
    LeaveCriticalSection(&s_chan);
    if (s_periodic_thread) {
        InterlockedExchange(&s_periodic_run, 0);
        WaitForSingleObject(s_periodic_thread, INFINITE);
        CloseHandle(s_periodic_thread); s_periodic_thread = NULL;
    }
    long rc = STATUS_NOERROR;
    for (int i = 0; i < DEV_MAX_PERIODIC; i++) {
        long st = periodic_stop_entry(i);
        if (rc == STATUS_NOERROR && st != STATUS_NOERROR) rc = st;
    }
    return rc;
}

static long periodic_clear_chan(uint8_t chan)
{
    EnterCriticalSection(&s_chan);
    memset(&s_repeat[chan], 0, sizeof(repeat_t));
    repeat_set_awaiting_locked(chan, false);
    LeaveCriticalSection(&s_chan);
    long rc = STATUS_NOERROR;
    for (int i = 0; i < DEV_MAX_PERIODIC; i++) {
        EnterCriticalSection(&s_per);
        bool match = s_periodic[i].in_use && s_periodic[i].chan == chan;
        LeaveCriticalSection(&s_per);
        if (match) {
            long st = periodic_stop_entry(i);
            if (rc == STATUS_NOERROR && st != STATUS_NOERROR) rc = st;
        }
    }
    return rc;
}

static long do_start_periodic(const uint8_t *p, uint16_t plen,
                              uint8_t *resp, uint16_t *rl, uint16_t cap)
{
    uint32_t wire_id = rd32le(p), interval = rd32le(p + 4), txflags = rd32le(p + 8);
    uint16_t len = (uint16_t)(p[12] | (p[13] << 8));
    if (wire_id == 0 || wire_id > DEV_MAX_CHANNELS || !dev_channel_find(wire_id))
        return ERR_INVALID_CHANNEL_ID;
    if (14 + (int)len > plen || len == 0 || len > 12) return ERR_INVALID_MSG;
    if (interval < 5 || interval > 65535) return ERR_INVALID_TIME_INTERVAL;

    uint8_t chan = (uint8_t)(wire_id - 1);
    /* Firmware timers only where the capture proved the record, and only for a
     * message the record can carry losslessly -- see fw_periodic_eligible().
     * Everything else uses the host scheduler, which is slower but complete. */
    uint16_t per_engine = proto_to_engine(chan_protocol(wire_id));
    bool use_fw = !periodic_force_host() &&
                  fw_periodic_eligible(per_engine, txflags, len);
    if (!use_fw && !periodic_force_host() && engine_is_can(per_engine) &&
        dev_log_enabled())
        dev_log("periodic ch=%u on host scheduler (engine=%04X txflags=0x%08lX "
                "len=%u): outside the firmware timer record's proven ground",
                chan, per_engine, (unsigned long)txflags, len);

    uint32_t new_id = 0; int fwslot = -1;
    uint8_t fw_data[12]; uint16_t fw_len = 0;
    long rc = ERR_EXCEEDED_LIMIT;
    EnterCriticalSection(&s_per);
    for (int i = 0; i < DEV_MAX_PERIODIC; i++) if (!s_periodic[i].in_use) {
        if (use_fw) {
            for (int s = 0; s < VCX_FW_PERIODIC_SLOTS; s++)
                if (!(s_fw_slot_used[chan] & (1u << s))) { fwslot = s; break; }
            if (fwslot < 0) break;   /* device slots exhausted -> ERR_EXCEEDED_LIMIT */
        }
        periodic_t *entry = &s_periodic[i];
        entry->in_use = true; entry->chan = chan;
        entry->id = ++s_periodic_seq; entry->interval = interval;
        entry->next = GetTickCount(); entry->txflags = txflags; entry->len = len;
        memcpy(entry->data, p + 14, len);
        entry->fw = use_fw; entry->fw_slot = (uint8_t)(fwslot < 0 ? 0 : fwslot);
        if (use_fw) {
            s_fw_slot_used[chan] |= (uint16_t)(1u << fwslot);
            memcpy(fw_data, entry->data, len); fw_len = len;
        }
        new_id = entry->id;
        if (resp && cap >= 4) { wr32le(resp, entry->id); if (rl) *rl = 4; }
        rc = STATUS_NOERROR;
        break;
    }
    LeaveCriticalSection(&s_per);
    if (rc != STATUS_NOERROR) return rc;

    if (use_fw) {
        long st = fw_periodic_add(chan, (uint8_t)fwslot, interval, fw_data, fw_len);
        if (st != STATUS_NOERROR) {
            /* ADD or ENABLE may have applied before a lost reply. Keep the
             * reservation if rollback also fails, so CLEAR/Close can retry. */
            for (int i = 0; i < DEV_MAX_PERIODIC; i++)
                if (s_periodic[i].in_use && s_periodic[i].id == new_id) {
                    long cleanup = periodic_stop_entry(i);
                    if (cleanup != STATUS_NOERROR)
                        dev_log("periodic startup failed; slot %d on channel %u retained for cleanup (%ld)",
                                fwslot, chan, cleanup);
                    break;
                }
            return st;
        }
        if (dev_log_enabled())
            dev_log("periodic id=%lu -> firmware timer ch=%u slot=%d interval=%lums",
                    (unsigned long)new_id, chan, fwslot, (unsigned long)interval);
        return STATUS_NOERROR;
    }

    /* Host scheduler path (K-line/J1850, or forced). */
    if (!s_periodic_thread) {
        InterlockedExchange(&s_periodic_run, 1);
        s_periodic_thread = CreateThread(NULL, 0, periodic_proc, NULL, 0, NULL);
        if (!s_periodic_thread) {
            /* Roll back only THIS request.  periodic_stop_all() used to be
             * called here, which tore down every other channel's periodics too
             * -- including firmware-timer entries that belong to callers who
             * are not failing, whose slots would then be unreachable from the
             * host while the device kept transmitting them. */
            InterlockedExchange(&s_periodic_run, 0);
            EnterCriticalSection(&s_per);
            for (int i = 0; i < DEV_MAX_PERIODIC; i++)
                if (s_periodic[i].in_use && s_periodic[i].id == new_id) {
                    memset(&s_periodic[i], 0, sizeof(s_periodic[i]));
                    break;
                }
            LeaveCriticalSection(&s_per);
            return ERR_FAILED;
        }
    }
    return STATUS_NOERROR;
}

static long do_stop_periodic(const uint8_t *p)
{
    uint32_t wire_id = rd32le(p), id = rd32le(p + 4);
    if (wire_id == 0 || wire_id > DEV_MAX_CHANNELS || !dev_channel_find(wire_id))
        return ERR_INVALID_CHANNEL_ID;
    uint8_t chan = (uint8_t)(wire_id - 1);
    for (int i = 0; i < DEV_MAX_PERIODIC; i++)
        if (s_periodic[i].in_use && s_periodic[i].id == id &&
            s_periodic[i].chan == chan)
            return periodic_stop_entry(i);
    return ERR_INVALID_MSG_ID;
}

static long do_version(uint8_t *resp, uint16_t *rl, uint16_t cap)
{
    if (rl) *rl = 0;
    if (!dev_is_connected()) return -1;
    if (!s_have_devinfo || cap < sizeof(s_devinfo)) return ERR_FAILED;
    memcpy(resp, s_devinfo, sizeof(s_devinfo));
    if (rl) *rl = sizeof(s_devinfo);
    return STATUS_NOERROR;
}

/* PassThruSetProgrammingVoltage -> firmware VCX_CmdDevSetPinVol (op 0x85).
 * payload (from api.c): [PinNumber u32 LE][Voltage u32 LE].
 * Firmware frame: msg[8]=pin, msg[9..12]=voltage as a big-endian u32. The
 * firmware honours the J2534 sentinels directly (0xFFFFFFFF=OFF,
 * 0xFFFFFFFE=SHORT_TO_GROUND); any other value is nominally millivolts.
 *
 * On THIS (Ford/Mazda branded hardware) unit the only wired rail is pin 13/FEPS and it is a
 * hard on/off at ~18 V -- see the capability warning block up top before
 * reusing any of this for another manufacturer's variant.  What we actually
 * attempt, and what status we return, is gated by s_prog_policy. */
static long release_prog_voltage(void)
{
    uint8_t off[5] = { (uint8_t)VCX_FEPS_PIN, 0xFF, 0xFF, 0xFF, 0xFF };
    long st = vcx_xact(0x00, 0x85, 0, off, sizeof(off), NULL, NULL, 0);
    if (st != VCX_STATUS_OK) {
        dev_log("programming voltage OFF unconfirmed (%ld); cleanup must be retried", st);
        return st < 0 ? -1 : ERR_FAILED;
    }
    s_prog_maybe_on = false;
    s_prog_pin = 0;
    s_prog_target_mv = 0;
    return STATUS_NOERROR;
}

static long do_set_prog_voltage(const uint8_t *p, uint16_t plen)
{
    if (plen < 8) return ERR_INVALID_MSG;
    uint32_t pin  = rd32le(p);
    uint32_t volt = rd32le(p + 4);

    /* "off" = the J2534 OFF sentinel or a plain 0. */
    bool want_off = (volt == 0 || volt == J2534_VOLTAGE_OFF);
    bool want_gnd = (volt == J2534_SHORT_TO_GND);
    bool is_feps  = pin_can_source(pin);

    /* Truthful policies reject every value on an unsupported pin consistently:
     * invalid for a non-J2534 programming-voltage pin, unsupported for a legal
     * but unwired pin.  Pin 0 + OFF is the exception: firmware unconditionally
     * drops the rail GPIO, making it a valid "release all" teardown request. */
    /* Always send release-all before policy gating: another process or session
     * may have left the rail high, and s_prog_pin tracks only this process. */
    if (pin == 0 && want_off) {
        return release_prog_voltage();
    }

    if (PROG_V_TRUTHFUL(s_prog_policy) && !pin_is_prog_voltage_arg(pin)) {
        dev_log("  set prog voltage pin=%lu volt=0x%08lx -> ERR_PIN_INVALID "
                "(not a programming-voltage pin in J2534-1)",
                (unsigned long)pin, (unsigned long)volt);
        return ERR_PIN_INVALID;
    }
    if (PROG_V_TRUTHFUL(s_prog_policy) && !is_feps) {
        dev_log("  set prog voltage pin=%lu volt=0x%08lx -> ERR_NOT_SUPPORTED "
                "(legal pin, but this unit cannot source there at any value)",
                (unsigned long)pin, (unsigned long)volt);
        return ERR_NOT_SUPPORTED;
    }

    /* loose/compat reaching a non-FEPS pin: no rail is wired there, so skip the
     * device round-trip and report the success the caller was promised. */
    if (!is_feps) {
        s_prog_pin = 0; s_prog_target_mv = 0;
        dev_log("  set prog voltage pin=%lu volt=0x%08lx -> ok "
                "(no rail on this pin; policy=%d)",
                (unsigned long)pin, (unsigned long)volt, (int)s_prog_policy);
        return STATUS_NOERROR;
    }

    /* ---- pin 13: a pin that genuinely sources, so the value now matters ---- */

    /* SHORT_TO_GROUND: the pin is valid, the operation is not --
     * ERR_NOT_SUPPORTED, not ERR_PIN_INVALID.  (Before this, 0xFFFFFFFE fell
     * through the want_off test and energised the rail to 18 V, the opposite of
     * what was asked.)
     *
     * Firmware analysis turned this from a plausible refusal into a required
     * one.  Grounding is a switch-fabric operation: op 0x85 / 0xFFFFFFFE
     * reaches bus_prog_gnd(), which only toggles I2C expander bits, and the
     * function that flushes those bits to the expanders (bus_mux_out) is
     * `bx lr` in this build -- so the request reaches no hardware at all.
     * Worse for a pass-through: bus_prog_gnd() is the one path that never calls
     * bsp_vddp_set(), so asking to ground an energised pin would leave ~18 V
     * standing on it while the device answered OK.  Refusing is the only answer
     * that does not mislead.  notes/firmware_power_architecture.md sec. 4.3. */
    if (want_gnd) {
        if (PROG_V_TRUTHFUL(s_prog_policy)) {
            dev_log("  set prog voltage pin=13 SHORT_TO_GROUND -> NOT_SUPPORTED "
                    "(grounding is fabric-only; fabric flush is stubbed in fw, "
                    "and it would not drop the rail)");
            return ERR_NOT_SUPPORTED;
        }
        /* loose/compat: claim success, but never energise a rail the caller
         * asked to ground. */
        dev_log("  set prog voltage pin=13 SHORT_TO_GROUND -> ok "
                "(not performed; policy=%d)", (int)s_prog_policy);
        return STATUS_NOERROR;
    }

    /* A specific millivolt level the fixed rail cannot produce.  Truthful modes
     * refuse rather than silently substituting ~18 V. */
    if (!want_off && PROG_V_TRUTHFUL(s_prog_policy) &&
        (volt < VCX_FEPS_MIN_MV || volt > VCX_FEPS_MAX_MV)) {
        dev_log("  set prog voltage pin=13 %lu mV -> NOT_SUPPORTED "
                "(rail is a fixed ~%u mV switch, not a DAC)",
                (unsigned long)volt, (unsigned)VCX_FEPS_NOMINAL_MV);
        return ERR_NOT_SUPPORTED;
    }

    /* Hard on/off.  Any accepted non-off request comes out as ~18 V, so send
     * the firmware OFF sentinel or a fixed 18 V rather than a millivolt value
     * the rail cannot honour.
     *
     * !!! The is_feps gate above is load-bearing, not decorative.  On the
     * !!! device the pin argument to op 0x85 is INERT for the rail: it only
     * !!! selects a switch-fabric bit that never reaches hardware, while
     * !!! bus_prog_vol()'s call to bsp_vddp_set(1) is unconditional.  Any pin
     * !!! number sent here with a non-sentinel value energises the one rail.
     * !!! Do not relax the gate so a non-13 request reaches this xact --
     * !!! "set 5 V on pin 6" would put ~18 V on pin 13.  Every policy,
     * !!! loose and compat included, must keep returning before this point for
     * !!! a non-FEPS pin.  notes/firmware_power_architecture.md sec. 4.2. */
    uint32_t hw_volt = want_off ? J2534_VOLTAGE_OFF : VCX_FEPS_NOMINAL_MV;
    uint8_t body[5] = { (uint8_t)VCX_FEPS_PIN,
                        (uint8_t)(hw_volt >> 24), (uint8_t)(hw_volt >> 16),
                        (uint8_t)(hw_volt >> 8),  (uint8_t)hw_volt };
    if (!want_off) s_prog_maybe_on = true;
    long st = vcx_xact(0x00, 0x85, 0, body, sizeof(body), NULL, NULL, 0);
    if (st < 0) return -1;
    if (st != VCX_STATUS_OK) return ERR_FAILED;
    s_prog_maybe_on = !want_off;
    s_prog_pin       = want_off ? 0 : (uint8_t)VCX_FEPS_PIN;
    s_prog_target_mv = want_off ? 0 : (volt == J2534_VOLTAGE_OFF ? VCX_FEPS_NOMINAL_MV : volt);
    dev_log("  set prog voltage pin=13 (FEPS) %s -> ok",
            want_off ? "OFF" : "ON(~18V)");
    return STATUS_NOERROR;
}

static long do_ioctl(const uint8_t *p, uint16_t plen, uint8_t *resp, uint16_t *rl, uint16_t cap)
{
    /* p: ChannelID(4) IoctlID(4) [NumOfParams(4) params...] */
    uint32_t wire = rd32le(p), id = rd32le(p + 4);
    int ch = (wire >= 1 && wire <= DEV_MAX_CHANNELS) ? (int)(wire - 1) : 0;

    switch (id) {
    case J2534_IOCTL_CLEAR_RX_BUFFER:
        return dev_channel_clear_rx(wire, NULL);
    case J2534_IOCTL_CLEAR_TX_BUFFER:
        if (!dev_channel_find(wire)) return ERR_INVALID_CHANNEL_ID;
        /* Firmware 4B/4C (08025FFE/0802601C) only log and return success.
         * Writes are synchronous on the host; submitted device TX cannot be
         * cancelled here. Keep the vendor no-op in HDS compatibility mode,
         * but never call it a device flush or reset filters/timers implicitly. */
        dev_log("  CLEAR_TX_BUFFER ch=%lu: device cancellation unavailable (firmware stub)",
                (unsigned long)wire);
        return dev_strict_validation() ? ERR_NOT_SUPPORTED : STATUS_NOERROR;
    case J2534_IOCTL_CLEAR_PERIODIC_MSGS:
        if (!dev_channel_find(wire)) return ERR_INVALID_CHANNEL_ID;
        return periodic_clear_chan((uint8_t)ch);
    case J2534_IOCTL_CLEAR_MSG_FILTERS:
        return do_clear_filters(wire);

    /* J1850 PWM/VPW functional-address table.  The firmware only carries one
     * functional address per engine (VCX_PID_J1850_FUNCT_ADDR, alongside node
     * address 0x8501 -- see the setters at 0x0803690C/0x08036E7A), so ADD
     * replaces it and DELETE/CLEAR reset it to 0 (no functional filtering).
     * That matches how HDS's J1850 PWM probe uses this table: one call, one
     * address, never a real list. */
    case J2534_IOCTL_ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
    case J2534_IOCTL_DELETE_FROM_FUNCT_MSG_LOOKUP_TBL:
    case J2534_IOCTL_CLEAR_FUNCT_MSG_LOOKUP_TABLE: {
        uint32_t val = 0;
        if (id != J2534_IOCTL_CLEAR_FUNCT_MSG_LOOKUP_TABLE) {
            if (plen < 12) return ERR_INVALID_MSG;
            if (id == J2534_IOCTL_ADD_TO_FUNCT_MSG_LOOKUP_TABLE) val = rd32le(p + 8);
        }
        uint8_t blob[6]; int bn = 0;
        par_emit(blob, &bn, sizeof(blob), VCX_PID_J1850_FUNCT_ADDR, val);
        long st = vcx_xact(0x00, VCX_OP_PARAMS, (uint8_t)ch, blob, bn, NULL, NULL, 0);
        if (st < 0) return -1;
        if (st != VCX_STATUS_OK) return ERR_FAILED;
        return STATUS_NOERROR;
    }

    case J2534_IOCTL_READ_VBATT: {
        uint8_t pin = 0x10, r[8]; uint16_t rn = 0;   /* OBD pin 16 = battery */
        long st = vcx_xact(0x00, 0x86, 0, &pin, 1, r, &rn, sizeof(r));
        if (st < 0) return -1;
        if (st != VCX_STATUS_OK) return ERR_FAILED;
        if (rn < 4) return ERR_FAILED;
        uint32_t mv = ((uint32_t)r[0]<<24)|((uint32_t)r[1]<<16)|(r[2]<<8)|r[3];
        if (resp && cap >= 4) { resp[0]=(uint8_t)mv; resp[1]=(uint8_t)(mv>>8);
                                resp[2]=(uint8_t)(mv>>16); resp[3]=(uint8_t)(mv>>24); if (rl) *rl = 4; }
        return STATUS_NOERROR;
    }
    case J2534_IOCTL_READ_PROG_VOLTAGE: {
        /* On THIS unit the one pin that can source programming voltage (13/FEPS)
         * has no sense path -- only pin 16/VBATT can be measured -- so there is
         * no honest numeric answer.  (A different-OEM variant with a real sense
         * channel on its programming pin would want to actually measure here;
         * see the capability warning block up top.)  What we return is policy:
         *   strict        -> NOT_SUPPORTED (be truthful: no readback exists)
         *   honest        -> the commanded rail state (0 or ~18000 mV), which
         *                    is knowable without measuring and is never an
         *                    arbitrary number the app fed us; a set-then-verify
         *                    flow completes instead of dying on the verify
         *   loose         -> the real measurement (~0 mV), whatever it is
         *   compat        -> echo the last requested target so rail checks pass */
        switch (s_prog_policy) {
        case PROG_V_STRICT:
            return ERR_NOT_SUPPORTED;
        case PROG_V_HONEST: {
            /* Commanded state, not a measurement: only pin 13 can ever be
             * energised here and it is on/off, so "on" is exactly 18000 mV.
             * Unlike compat this never echoes a level the rail cannot make. */
            uint32_t mv = (s_prog_pin == VCX_FEPS_PIN) ? VCX_FEPS_NOMINAL_MV : 0u;
            if (resp && cap >= 4) { resp[0]=(uint8_t)mv; resp[1]=(uint8_t)(mv>>8);
                                    resp[2]=(uint8_t)(mv>>16); resp[3]=(uint8_t)(mv>>24);
                                    if (rl) *rl = 4; }
            dev_log("  READ_PROG_VOLTAGE -> %lu mV (commanded state, honest policy)",
                    (unsigned long)mv);
            return STATUS_NOERROR;
        }
        case PROG_V_COMPAT: {
            uint32_t mv = s_prog_target_mv;
            if (resp && cap >= 4) { resp[0]=(uint8_t)mv; resp[1]=(uint8_t)(mv>>8);
                                    resp[2]=(uint8_t)(mv>>16); resp[3]=(uint8_t)(mv>>24);
                                    if (rl) *rl = 4; }
            return STATUS_NOERROR;
        }
        case PROG_V_LOOSE:
        default: {
            /* Sample the pin last set via the same GetPinVol (0x86) the battery
             * read uses.  No pin set / OFF -> 0 mV; pin 13 has no sense route,
             * so expect ~0 mV back even after a set -- and that is the point. */
            uint8_t pin = s_prog_pin, r[8]; uint16_t rn = 0;
            if (pin == 0) { if (resp && cap >= 4) { resp[0]=resp[1]=resp[2]=resp[3]=0; if (rl) *rl = 4; }
                            return STATUS_NOERROR; }
            long st = vcx_xact(0x00, 0x86, 0, &pin, 1, r, &rn, sizeof(r));
            if (st < 0) return -1;
            if (st != VCX_STATUS_OK) return ERR_FAILED;
            if (rn < 4) return ERR_FAILED;
            uint32_t mv = ((uint32_t)r[0]<<24)|((uint32_t)r[1]<<16)|(r[2]<<8)|r[3];
            if (resp && cap >= 4) { resp[0]=(uint8_t)mv; resp[1]=(uint8_t)(mv>>8);
                                    resp[2]=(uint8_t)(mv>>16); resp[3]=(uint8_t)(mv>>24); if (rl) *rl = 4; }
            return STATUS_NOERROR;
        }
        }
    }
    case J2534_IOCTL_GET_CONFIG: {
        if (!dev_channel_find(wire)) return ERR_INVALID_CHANNEL_ID;
        uint32_t n = (plen >= 12) ? rd32le(p + 8) : 0;
        uint32_t avail = (plen > 12) ? (uint32_t)(plen - 12) / 4 : 0;
        if (n > avail) n = avail;
        for (uint32_t i = 0; i < n && (i + 1) * 8 <= cap; i++) {
            uint32_t param = rd32le(p + 12 + i * 4), val = cfg_get(ch, param);
            if ((param & 0xFFFF0000u) == 0x10000u) {
                long vst = pt_validate_config(chan_protocol(wire), param, val, true);
                if (vst != STATUS_NOERROR) return vst;
            }
            resp[i*8+0]=(uint8_t)param; resp[i*8+1]=(uint8_t)(param>>8);
            resp[i*8+2]=(uint8_t)(param>>16); resp[i*8+3]=(uint8_t)(param>>24);
            resp[i*8+4]=(uint8_t)val; resp[i*8+5]=(uint8_t)(val>>8);
            resp[i*8+6]=(uint8_t)(val>>16); resp[i*8+7]=(uint8_t)(val>>24);
            if (rl) *rl = (uint16_t)((i + 1) * 8);
        }
        return STATUS_NOERROR;
    }
    case J2534_IOCTL_SET_CONFIG: {
        if (!dev_channel_find(wire)) return ERR_INVALID_CHANNEL_ID;
        uint32_t n = (plen >= 12) ? rd32le(p + 8) : 0;
        uint32_t avail = (plen > 12) ? (uint32_t)(plen - 12) / 8 : 0;
        if (n > avail) return ERR_INVALID_IOCTL_VALUE;
        if (n > 40) return ERR_EXCEEDED_LIMIT;
        cfg_kv saved_cfg[40];
        memcpy(saved_cfg, s_cfg[ch], sizeof(saved_cfg));
        int saved_n = s_cfg_n[ch];
        uint32_t saved_pins = s_pins[ch];
        bool saved_pins_set = s_pins_set[ch];
        uint8_t saved_uart_pin = s_uart_pin[ch];
        long result = STATUS_NOERROR;
        bool device_uncertain = false;
        uint8_t blob[6 * 84]; int bn = 0;
        bool repin = false;
        /* Validate the WHOLE list before caching any of it.  SET_CONFIG is one
         * call, so it has to succeed or fail as one: validating inside the apply
         * loop meant an entry rejected halfway left every earlier entry sitting
         * in s_cfg while the early return skipped the VCX_OP_PARAMS push below,
         * so GET_CONFIG reported values the engine had never been given -- and a
         * later pin-change repin replayed the whole stale cache at it. */
        for (uint32_t i = 0; i < n; i++) {
            uint32_t param = rd32le(p + 12 + i*8), val = rd32le(p + 16 + i*8);
            long vst = pt_validate_config(chan_protocol((uint32_t)ch + 1), param,
                                          val, dev_strict_validation());
            if (vst != STATUS_NOERROR) {
                dev_log("  SET_CONFIG param=0x%lX value=%lu rejected (0x%02lX); "
                        "no entry in this call was applied",
                        (unsigned long)param, (unsigned long)val, vst);
                return vst;
            }
        }
        for (uint32_t i = 0; i < n; i++) {
            uint32_t param = rd32le(p + 12 + i*8), val = rd32le(p + 16 + i*8);
            if (param == J2534_CFG_P1_MAX && val == 0) {
                /* Out of J2534 range; see KLINE_P1_MAX_DEFAULT_MS. */
                dev_log("  SET_CONFIG P1_MAX=0 is outside J2534's 1..0xFFFF; "
                        "restoring the %u ms default", KLINE_P1_MAX_DEFAULT_MS);
                val = KLINE_P1_MAX_DEFAULT_MS;
            }
            bool found = false;
            for (int k = 0; k < s_cfg_n[ch]; k++) if (s_cfg[ch][k].param == param) found = true;
            if (!found && s_cfg_n[ch] == 40) { result = ERR_EXCEEDED_LIMIT; goto config_failed; }
            cfg_set(ch, param, val);
            if (param == J2534_2_CFG_J1962_PINS) {
                /* This is how a _PS channel gets told which OBD pins to use --
                 * for FORScan on a Ford, 0x030B to reach MS-CAN and the modules
                 * that live there (instrument cluster, body modules). */
                uint32_t pins = val & 0xFFFFu;
                uint16_t pin_engine = proto_to_engine(chan_protocol((uint32_t)ch + 1));
                if (engine_is_kline(pin_engine)) {
                    /* A UART engine reads only pin1 (J1962_PINS bits 15..8);
                     * the vendor DLL leaves pin2 zero for every UART protocol,
                     * and uart_hw_set_pin's hardwired-K-line test is
                     * `pin1 == 7 && pin2 == 0`.  So HDS asking for 0700 --
                     * UART_ECHO_BYTE_PS does -- is asking for what bring-up
                     * already claimed, and 0E00/0100/0F00 (pins 14/1/15, the
                     * ones HDS's HONDA_DIAGH_PS and ISO9141_PS probes want)
                     * are pin1 = 14/1/15 with no pin2.
                     *
                     * Those alternates need routing_matrix_build, whose commit
                     * routine (0x08028E28) is two bytes of `bx lr` in this
                     * image and whose I2C expander bank a Nano does not carry.
                     * The vendor DLL forwards them anyway and reports success,
                     * so a genuine Nano answers the ioctl and then hears
                     * nothing; refusing is the more honest of the two, and
                     * kline_pin=any in vcx_nano.ini forwards them for bench
                     * work.  The CAN bus/pin comm-params below are meaningless
                     * to a UART engine and must not be pushed. */
                    uint8_t pin1 = (uint8_t)((pins >> 8) & 0xFFu);
                    uint8_t pin2 = (uint8_t)(pins & 0xFFu);
                    /* Compare against the pin this channel actually claimed
                     * (uart_pin_default() -- 7 unless overridden by kline_pin=N
                     * in vcx_nano.ini), not the bare hardware default: otherwise
                     * an app asking for the ordinary pin 7 while an operator's
                     * kline_pin=N override is in effect would sail past this
                     * check and silently clobber the override below. */
                    if (pins != 0 && !(pin1 == s_uart_pin[ch] && pin2 == 0) &&
                        !kline_pin_any()) {
                        dev_log("  J1962_PINS 0x%04lX: this device drives only "
                                "pin %u for K-line/UART; pin %u needs the "
                                "routing matrix, whose commit routine is a stub "
                                "(kline_pin=any to forward it anyway)",
                                (unsigned long)pins, (unsigned)s_uart_pin[ch],
                                (unsigned)pin1);
                        result = ERR_NOT_SUPPORTED; goto config_failed;
                    }
                    /* s_pins[]/s_pins_set[] are the CAN pin-change cache below;
                     * a kline channel's repin decision is s_uart_pin[]-driven
                     * (next), and GET_CONFIG answers from the separate s_cfg
                     * cache (already updated by cfg_set() above) -- nothing
                     * reads s_pins[]/s_pins_set[] for a kline channel. */
                    if (pins != 0 && pin1 != s_uart_pin[ch]) {
                        /* Only reachable with kline_pin=any: re-latch the pin,
                         * which the engine samples at start (see the repin
                         * path below). */
                        s_uart_pin[ch] = pin1;
                        repin = true;
                        dev_log("  J1962_PINS 0x%04lX -> UART pin %u "
                                "(forwarded, route not verified)",
                                (unsigned long)pins, (unsigned)pin1);
                    } else {
                        dev_log("  J1962_PINS 0x%04lX accepted -> pin %u "
                                "(K-line, hardwired)", (unsigned long)pins,
                                (unsigned)s_uart_pin[ch]);
                    }
                    continue;
                }
                if (!engine_is_can(pin_engine)) { result = ERR_NOT_SUPPORTED; goto config_failed; }
                uint32_t bus = 0;
                bool forced = false;
                bool send_bus = can_bus_for_pins(pins, &bus, &forced);
                if (!forced && pins != 0 && pins != VCX_PINS_HS_CAN &&
                    pins != VCX_PINS_MS_CAN) {
                    dev_log("  J1962_PINS 0x%04lX: only 060E (6/14) and 030B "
                            "(3/11) are wired on this device; the routing "
                            "matrix it would need is not implemented",
                            (unsigned long)pins);
                    result = ERR_NOT_SUPPORTED; goto config_failed;
                }
                par_emit(blob, &bn, sizeof(blob), VCX_PID_BUS_PIN, val << 16);
                if (send_bus) {
                    par_emit(blob, &bn, sizeof(blob), VCX_PID_CAN_BUS, bus);
                    dev_log("  J1962_PINS 0x%04lX -> pins %lu/%lu on CAN%lu",
                            (unsigned long)(val & 0xFFFF), (unsigned long)((val >> 8) & 0xFF),
                            (unsigned long)(val & 0xFF), (unsigned long)bus + 1);
                } else {
                    dev_log("  J1962_PINS 0x%04lX -> pins %lu/%lu, controller left to firmware",
                            (unsigned long)(val & 0xFFFF), (unsigned long)((val >> 8) & 0xFF),
                            (unsigned long)(val & 0xFF));
                }
                if (!s_pins_set[ch] || s_pins[ch] != val) {
                    s_pins[ch] = val; s_pins_set[ch] = true; repin = true;
                }
                /* The baud belongs in the same push: a restarted channel should
                 * come up fully configured, exactly as do_connect brings it up. */
                par_emit(blob, &bn, sizeof(blob), VCX_PID_BAUDRATE,
                         cfg_get(ch, J2534_CFG_DATA_RATE));
                continue;
            }
            if (param == J2534_CFG_DATA_BITS || param == J2534_CFG_PARITY) {
                uint16_t engine = proto_to_engine(chan_protocol((uint32_t)ch + 1));
                if (!engine_is_kline(engine)) continue;
                par_emit(blob, &bn, sizeof(blob), VCX_PID_UART_FORMAT,
                         uart_format_value(ch));
                continue;
            }
            uint16_t vid = j2534_to_vcx_param(proto_to_engine(chan_protocol((uint32_t)ch + 1)), param);
            if (vid) par_emit(blob, &bn, sizeof(blob), vid, j2534_to_vcx_value(param, val));
        }
        if (repin) {
            bool busy = s_filt_n[ch] != 0;
            EnterCriticalSection(&s_chan);
            /* Completed repeats retain their record until explicit STOP. */
            if (s_repeat[ch].used) busy = true;
            for (int k = 0; k < VCX_MAX_BLOCK; k++) if (s_block[ch][k].in_use) busy = true;
            LeaveCriticalSection(&s_chan);
            EnterCriticalSection(&s_per);
            for (int k = 0; k < DEV_MAX_PERIODIC; k++)
                if (s_periodic[k].in_use && s_periodic[k].chan == ch) busy = true;
            LeaveCriticalSection(&s_per);
            if (busy) { result = ERR_CHANNEL_IN_USE; goto config_failed; }
            /* Close and reopen so the parameters land before the channel's
             * first start -- see chan_bring_up.  A fresh open resets every
             * comm-param, so the whole cached config goes out with them, not
             * just the entries in this SET_CONFIG call. */
            uint16_t engine = proto_to_engine(chan_protocol((uint32_t)ch + 1));
            if (!engine) { result = ERR_INVALID_CHANNEL_ID; goto config_failed; }
            uint8_t all[6 * 44]; int an = 0;
            bool uart_format_emitted = false;
            for (int i = 0; i < s_cfg_n[ch]; i++) {
                uint32_t prm = s_cfg[ch][i].param, v = s_cfg[ch][i].value;
                if (prm == J2534_2_CFG_J1962_PINS) {
                    /* A UART engine's pin is replayed from s_uart_pin below --
                     * as pin1 only, and whether or not the application ever
                     * sent J1962_PINS.  The CAN bus/pin params do not apply,
                     * matching the SET path above. */
                    if (engine_is_kline(engine)) continue;
                    uint32_t b = 0; bool f = false;
                    par_emit(all, &an, sizeof(all), VCX_PID_BUS_PIN, v << 16);
                    if (can_bus_for_pins(v & 0xFFFFu, &b, &f)) {
                        par_emit(all, &an, sizeof(all), VCX_PID_CAN_BUS, b);
                    }
                    continue;
                }
                if (prm == J2534_CFG_DATA_BITS || prm == J2534_CFG_PARITY) {
                    /* Emit the packed word once when rebuilding all config. */
                    if (engine_is_kline(engine) && !uart_format_emitted) {
                        par_emit(all, &an, sizeof(all), VCX_PID_UART_FORMAT,
                                 uart_format_value(ch));
                        uart_format_emitted = true;
                    }
                    continue;
                }
                uint16_t vid = j2534_to_vcx_param(engine, prm);
                if (vid) par_emit(all, &an, sizeof(all), vid, j2534_to_vcx_value(prm, v));
            }
            if (engine_is_kline(engine)) emit_kline_pin(all, &an, sizeof(all), s_uart_pin[ch]);
            /* The reopen resets every comm-param, so the channel-level connect
             * flags (checksum) go out again with the cached
             * config, exactly as do_connect first pushed them. */
            emit_connect_flag_params(all, &an, sizeof(all), engine, s_connect_flags[ch]);
            device_uncertain = true;
            vcx_xact(0x00, VCX_OP_STOP,  (uint8_t)ch, NULL, 0, NULL, NULL, 0);
            vcx_xact(0x00, VCX_OP_CLOSE, (uint8_t)ch, NULL, 0, NULL, NULL, 0);
            long st = chan_bring_up((uint8_t)ch, engine, all, an);
            if (st != STATUS_NOERROR) { result = st; goto config_failed; }
            s_filter_seq[ch] = 0;   /* the reopened channel has no filters */
            dev_log("  channel %d reopened on the new pins (%d params)", ch, an / 6);
            return STATUS_NOERROR;
        }
        if (bn > 0) {
            device_uncertain = true;
            long st = vcx_xact(0x00, VCX_OP_PARAMS, (uint8_t)ch, blob, bn, NULL, NULL, 0);
            if (st < 0) { result = -1; goto config_failed; }
            if (st != VCX_STATUS_OK) { result = ERR_FAILED; goto config_failed; }
        }
        return STATUS_NOERROR;
config_failed:
        /* 0x45 can partially apply before an error/timeout. Retire the channel
         * rather than claim the saved cache describes a still-live device. */
        if (device_uncertain) {
            periodic_clear_chan((uint8_t)ch);
            vcx_xact(0, VCX_OP_STOP, (uint8_t)ch, NULL, 0, NULL, NULL, 0);
            vcx_xact(0, VCX_OP_CLOSE, (uint8_t)ch, NULL, 0, NULL, NULL, 0);
            s_filt_n[ch] = 0;
            EnterCriticalSection(&s_chan);
            memset(s_block[ch], 0, sizeof(s_block[ch]));
            LeaveCriticalSection(&s_chan);
            dev_channel_remove((uint32_t)ch + 1);
            dev_log("  SET_CONFIG device state uncertain; channel retired, reconnect required");
        }
        memcpy(s_cfg[ch], saved_cfg, sizeof(saved_cfg));
        s_cfg_n[ch] = saved_n;
        s_pins[ch] = saved_pins;
        s_pins_set[ch] = saved_pins_set;
        s_uart_pin[ch] = saved_uart_pin;
        return result;
    }
    default:
        /* Defined-but-unhandled is NOT_SUPPORTED; only an id outside the
         * documented range is an invalid argument.  See api.c's default arm. */
        return j2534_ioctl_id_defined(id) ? ERR_NOT_SUPPORTED
                                          : ERR_INVALID_IOCTL_ID;
    }
}

static long dispatch_request(uint8_t type, const uint8_t *payload, uint16_t payload_len,
                 uint8_t *resp, uint16_t *resp_len, uint16_t resp_cap)
{
    switch (type) {
    case PT_CMD_PING:
    case PT_CMD_OPEN:      return dev_is_connected() ? STATUS_NOERROR : -1;
    case PT_CMD_RESET:                         /* deliberate no-op: never reboot device */
    case PT_CMD_CLOSE:     return STATUS_NOERROR;
    case PT_CMD_VERSION:   return do_version(resp, resp_len, resp_cap);
    case PT_CMD_CONNECT:   return do_connect(payload, resp, resp_len, resp_cap);
    case PT_CMD_DISCONNECT:return do_disconnect(payload);
    case PT_CMD_WRITE_MSG: return do_write(payload, payload_len);
    case PT_CMD_START_FILTER: return do_filter(payload, payload_len, resp, resp_len, resp_cap);
    case PT_CMD_STOP_FILTER:  return do_stop_filter(payload);
    case PT_CMD_IOCTL: return do_ioctl(payload, payload_len, resp, resp_len, resp_cap);
    case PT_CMD_SET_PROG_VOLTAGE: return do_set_prog_voltage(payload, payload_len);
    case PT_CMD_START_PERIODIC:
        return do_start_periodic(payload, payload_len, resp, resp_len, resp_cap);
    case PT_CMD_STOP_PERIODIC:
        return do_stop_periodic(payload);
    default: return ERR_NOT_SUPPORTED;
    }
}

long dev_request(uint8_t type, const uint8_t *payload, uint16_t payload_len,
                 uint8_t *resp, uint16_t *resp_len, uint16_t resp_cap)
{
    /* Serialize complete API mutations, including firmware transactions and
     * their host bookkeeping, against Close and cross-process handoff. The
     * reader and host scheduler do not take s_link, so replies still drain. */
    EnterCriticalSection(&s_link);
    long rc = dispatch_request(type, payload, payload_len, resp, resp_len, resp_cap);
    LeaveCriticalSection(&s_link);
    return rc;
}
