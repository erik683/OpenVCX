/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * device.h - serial client for the passthru wire protocol.
 *
 * Owns the COM port, a reader thread that decodes frames, sequence-matched
 * request/response handling, and per-channel receive queues fed by EVT_MSG
 * events. All functions are thread-safe.
 */
#pragma once

#include <windows.h>
#include <stdbool.h>
#include <stdint.h>

#define DEV_MAX_CHANNELS  4
#define DEV_RX_QUEUE_CAP  8192
#define DEV_RESP_CAP      4300
#define DEV_REQ_TIMEOUT_MS 3000

typedef struct rx_msg {
    struct rx_msg *next;
    uint32_t rx_status;
    uint32_t timestamp; /* host QPC microseconds (low 32 bits) when the serial read returned */
    uint16_t len;
    uint8_t  data[1]; /* allocated to len */
} rx_msg_t;

typedef struct {
    bool     in_use;
    uint32_t wire_id;
    uint32_t protocol;
    uint64_t generation; /* changes whenever this slot is reused */
    /* Owned by the fixed table slot, not an individual channel lifetime. */
    HANDLE   rx_event;     /* auto-reset, signaled on enqueue */
    rx_msg_t *head, *tail;
    int      count;
    bool     overflowed;   /* device or DLL queue overflow pending report */
} dev_channel_t;

void dev_init(HINSTANCE dll_module);
void dev_shutdown(void);

/* Locate the device (ini > env > COM scan), open the port, start the
 * reader. Fills err with a human-readable reason on failure.
 *
 * On failure, was_busy (if non-NULL) is set to true when the Nano's port is
 * held by another process that would not release it, rather than because no
 * device was found -- PassThruOpen reports that as ERR_DEVICE_IN_USE, which is
 * what it is.  Filled from inside dev_connect()'s own lock so the caller gets
 * an answer for *this* call, not whatever a concurrent dev_connect() left in a
 * shared flag.
 *
 * A warm link is not private to this process: while one is held, the port is
 * published under a session-local named event, and any other OpenVCX32.dll
 * instance that finds the port taken asks for it through that event.  The
 * holder gives it up whenever it has no J2534 session of its own open --
 * between PassThruClose and the next PassThruOpen -- which is the window HDS
 * uses when it launches DataListClient.exe or DTCMonitor.exe. */
bool dev_connect(char *err, size_t err_len, bool *was_busy);
void dev_disconnect(void);
/* End the logical session (PassThruClose). Keeps the physical COM link, reader
 * thread and licence warm for the next PassThruOpen unless keep_warm=0 is set
 * in vcx_nano.ini, in which case it tears the port down fully. A failure on a
 * live link retains session ownership so cleanup can be retried. */
long dev_session_close(void);
bool dev_is_connected(void);

/* Complete API mutations share the link lifecycle lock. Do not hold it while
 * waiting in ReadMsgs; use the channel snapshot/generation helpers instead. */
void dev_api_lock(void);
void dev_api_unlock(void);

/* Send one command and wait for its response. Returns the device's i32
 * status, or -1 on transport failure (timeout/unplugged). Response extras
 * (after the status field) are copied into resp when non-NULL. */
long dev_request(uint8_t type, const uint8_t *payload, uint16_t payload_len,
                 uint8_t *resp, uint16_t *resp_len, uint16_t resp_cap);

dev_channel_t *dev_channel_add(uint32_t wire_id, uint32_t protocol);
/* The returned pointer refers to a fixed table slot.  Re-resolve by wire ID
 * after a blocking wait: the slot may have been recycled in the meantime. */
dev_channel_t *dev_channel_find(uint32_t wire_id);
void dev_channel_remove(uint32_t wire_id);
void dev_channels_clear(void);

/* ISO9141 response-conditioned host repetition. Calls require dev_api_lock. */
long dev_repeat_start(uint32_t channel, uint32_t interval, uint32_t condition,
                      uint32_t flags, const uint8_t *tx, uint16_t tx_len,
                      const uint8_t *mask, const uint8_t *pattern, uint16_t filter_len,
                      uint32_t *id);
long dev_repeat_query(uint32_t channel, uint32_t id, uint32_t *active);
void dev_repeat_activate(uint32_t channel);
long dev_repeat_stop(uint32_t channel, uint32_t id);

/* Bracket one FIVE_BAUD_INIT: begin applies W5 / UEB_T0 as the pre-init idle
 * and returns the firmware's worst case in ms; end restores the idle and, after
 * a failure, re-enables firmware periodics. Calls require dev_api_lock. */
long dev_five_baud_begin(uint32_t wire_id, uint32_t *worst_ms);
void dev_five_baud_end(uint32_t wire_id, bool failed);
/* Low 32 bits of the host QueryPerformanceCounter clock in microseconds: the
 * clock rx_msg_t.timestamp is taken from. */
uint32_t dev_host_us(void);
/* FAST_INIT reply wait in ms, from the channel's timing (or the ini override),
 * and the earliest a reply can arrive after the request. Calls require
 * dev_api_lock. */
long dev_fast_init_window(uint32_t wire_id, uint16_t pdu_len,
                          uint32_t *timeout_ms, uint32_t *min_reply_ms);

/* Pop one message; returns NULL when empty (caller frees with free()). */
rx_msg_t *dev_channel_pop(dev_channel_t *ch);

/* Diagnostic log, enabled by the VCX_NANO_LOG env var or log=<path> in
 * vcx_nano.ini (next to the DLL). No-op otherwise; dev_log_enabled lets
 * callers skip formatting work on the hot receive path.
 *
 * Everything the log says must be true, and everything it drops must be
 * counted: analysis of this device is done from these files alone, so a
 * shortened dump always says how many bytes it stands for, and every discarded
 * byte, malformed frame and I/O error is recorded rather than swallowed. */
bool dev_log_enabled(void);
void dev_log(const char *fmt, ...);

/* Log a line ending in a hex dump of n bytes.  If the active hex_max caps it,
 * the header carries "shown=<k>" and the true length stays in "len=<n>" -- a
 * truncated dump is never silent. */
void dev_log_hex(const uint8_t *p, int n, const char *fmt, ...);

/* Commit buffered diagnostic output after a timing-sensitive operation. */
void dev_log_flush(void);

/* Level 1 keeps the per-call trace; level 2 (default) adds the high-frequency
 * entry/exit lines (ReadMsgs polls, WriteMsgs). Set with VCX_NANO_LOG_LEVEL or
 * log_level=<n> in vcx_nano.ini. */
bool dev_log_verbose(void);

/* Running totals of everything the transport dropped or failed on. Emitted at
 * session teardown and callable on demand. */
void dev_log_stats(const char *when);

/* J2534 validation strictness (strict_validation=<0|1> in vcx_nano.ini, or the
 * VCX_NANO_STRICT env var).  Default 0 = vendor-compatible: spec-illegal but
 * vendor-tolerated inputs are logged and forwarded.  1 = strict: they return the
 * spec's ERR_* instead.  Threaded into pt_validate_* by api.c and device_vcx.c. */
bool dev_strict_validation(void);

/* J1962 pin a UART/K-line channel drives (PID_BUS_PIN pin1); 0 for other
 * engines or an unknown channel. */
uint8_t dev_channel_uart_pin(uint32_t wire_id);

/* Atomic host RX boundary; generation is optional. No device-side reset. */
long dev_channel_clear_rx(uint32_t wire_id, uint64_t *generation);
/* Pop only from the captured channel lifetime; false means closed/replaced. */
bool dev_channel_pop_generation(uint32_t wire_id, uint64_t generation,
                                rx_msg_t **msg, HANDLE *event);
typedef struct {
    uint32_t protocol;
    uint64_t generation;
    HANDLE rx_event;
} dev_channel_snapshot_t;
bool dev_channel_snapshot(uint32_t wire_id, dev_channel_snapshot_t *snapshot);
bool dev_channel_take_overflow(uint32_t wire_id, uint64_t generation, bool *overflow);
