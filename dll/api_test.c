/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/* API regressions with real channel state and scripted transport. No COM I/O. */
#include <windows.h>
#include <stdbool.h>
#include <stdint.h>
static long config_test_xact(uint8_t, uint8_t, const uint8_t *, int);
static long init_test_send(unsigned long, uint32_t, const uint8_t *, uint16_t);
static bool test_connect(char *, size_t, bool *);
static long test_request(uint8_t, const uint8_t *, uint16_t, uint8_t *, uint16_t *, uint16_t);
static DWORD WINAPI test_wait(HANDLE, DWORD);
static long repeat_test_send(const uint8_t *, uint16_t);
static bool repeat_test_manual = true;
static DWORD repeat_test_now = 1000000; /* 0 = real GetTickCount */
#define VCX_REPEAT_TEST
#define VCX_CONFIG_TEST
#define VCX_INIT_TEST
#define PT_INIT_TIMEOUT_MS 10u
#include "device_vcx.c"
static void capture_api_log(const char *, ...);
#define dev_log capture_api_log
#define dev_log_enabled() true
#define dev_connect test_connect
#define dev_request test_request
#define WaitForSingleObject test_wait
#include "api.c"
#undef dev_connect
#undef dev_request
#undef WaitForSingleObject
#undef dev_log
#undef dev_log_enabled

static char api_log[100000];
static size_t api_log_used;
static bool capture_logging;
static void capture_api_log(const char *format, ...)
{
    if (!capture_logging || api_log_used + 2 >= sizeof(api_log)) return;
    va_list ap;
    va_start(ap, format);
    int n = vsnprintf(api_log + api_log_used, sizeof(api_log) - api_log_used, format, ap);
    va_end(ap);
    if (n < 0) return;
    size_t available = sizeof(api_log) - api_log_used - 2;
    api_log_used += (size_t)n > available ? available : (size_t)n;
    api_log[api_log_used++] = '\n';
    api_log[api_log_used] = 0;
}

static int failures, controls, writes, inits, delay_write, short_type, bad_open;
static uint16_t last_init_len, filter_flow_len;
static uint16_t init_baud = 10400;   /* sync baud in the stubbed five-baud record */
static bool init_silent;             /* stub pushes no five-baud record */
static bool block_open, block_connect, block_read;
static volatile LONG opens;
static HANDLE entered, release_call, entered_twice, close_done;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)

static struct { uint8_t op, ch, len, data[8]; } xacts[16];
static int xact_n;
static long config_test_xact(uint8_t op, uint8_t ch, const uint8_t *p, int n)
{
    if (xact_n < 16) {
        xacts[xact_n].op = op; xacts[xact_n].ch = ch; xacts[xact_n].len = (uint8_t)n;
        memcpy(xacts[xact_n].data, p, n < 8 ? (size_t)n : 8);
        xact_n++;
    }
    controls++;
    return 0;
}

static void push(uint32_t status)
{
    uint8_t bytes[5] = {0,0,7,0xE8,0x62};
    EnterCriticalSection(&s_chan);
    CHECK(chan_push_locked(&s_channels[0], status, bytes, sizeof(bytes), 0) == CHAN_PUSH_OK);
    LeaveCriticalSection(&s_chan);
}

static long init_test_send(unsigned long ch, uint32_t flag, const uint8_t *data, uint16_t len)
{
    (void)ch; (void)data;
    inits++; last_init_len = len;
    if (flag == 8 && !init_silent) {
        uint8_t keys[6] = {0x55,0x08,0x94,(uint8_t)(init_baud >> 8),(uint8_t)init_baud,0};
        EnterCriticalSection(&s_chan);
        CHECK(chan_push_locked(&s_channels[0], 0, keys, sizeof(keys), 0) == CHAN_PUSH_OK);
        LeaveCriticalSection(&s_chan);
    }
    return 0;
}

static bool test_connect(char *why, size_t size, bool *busy)
{
    (void)why; (void)size;
    *busy = false;
    LONG n = InterlockedIncrement(&opens);
    if (block_open) {
        SetEvent(entered);
        if (n > 1) SetEvent(entered_twice);
        CHECK(WaitForSingleObject(release_call, 2000) == WAIT_OBJECT_0);
    }
    s_session_active = true;
    return true;
}

static long test_request(uint8_t type, const uint8_t *p, uint16_t n,
                          uint8_t *resp, uint16_t *rn, uint16_t cap)
{
    if (bad_open && type == PT_CMD_OPEN) return ERR_FAILED;
    if (short_type == type) { if (rn) *rn = 0; return 0; }
    if (type == PT_CMD_WRITE_MSG) {
        writes++;
        if (delay_write) Sleep((DWORD)delay_write);
        return 0;
    }
    if (type == PT_CMD_START_FILTER) filter_flow_len = (uint16_t)(p[16] | p[17] << 8);
    long rc = dev_request(type, p, n, resp, rn, cap);
    if (block_connect && type == PT_CMD_CONNECT && rc == 0) {
        SetEvent(entered);
        CHECK(WaitForSingleObject(release_call, 2000) == WAIT_OBJECT_0);
    }
    return rc;
}

static DWORD WINAPI test_wait(HANDLE event, DWORD ms)
{
    if (block_read && event == s_channels[0].rx_event) {
        SetEvent(entered);
        CHECK(WaitForSingleObject(release_call, 2000) == WAIT_OBJECT_0);
    }
    return WaitForSingleObject(event, ms);
}

static void reset(uint32_t proto)
{
    dev_channels_clear();
    if (s_com != INVALID_HANDLE_VALUE) CloseHandle(s_com);
    s_com = CreateEvent(NULL, FALSE, FALSE, NULL);
    CHECK(s_com != NULL);
    memset(s_periodic, 0, sizeof(s_periodic)); memset(s_fw_slot_used, 0, sizeof(s_fw_slot_used));
    memset(s_cfg_n, 0, sizeof(s_cfg_n));
    dev_channel_add(1, proto);
    controls = writes = inits = delay_write = short_type = bad_open = 0;
    opens = 0; block_open = block_connect = block_read = false;
    g_open = 1; s_session_active = true;
    s_prog_maybe_on = false; s_prog_pin = 0;
    s_strict_validation = false;
}

static void repeat_capture_test(void)
{
    reset(J2534_CAN); /* Unsupported protocols retain the capture/error path. */
    repeat_log_input request = {0};
    CHECK(sizeof(request) == 0x30B0);
    CHECK((char *)&request.mask - (char *)&request == 0x1040);
    CHECK((char *)&request.pattern - (char *)&request == 0x2078);
    request.interval = 30; request.condition = 1;
    request.tx.ProtocolID = request.mask.ProtocolID = request.pattern.ProtocolID = 3;
    request.tx.TxFlags = 0x200;
    request.tx.DataSize = 300;
    for (unsigned i = 0; i < 300; ++i) request.tx.Data[i] = (uint8_t)i;
    request.mask.DataSize = request.pattern.DataSize = 2;
    request.mask.Data[1] = 0xFF; request.pattern.Data[1] = 0x78;
    unsigned long output = 0xAABBCCDD, message_id = 42;
    api_log_used = 0; api_log[0] = 0; capture_logging = true;
    CHECK(PassThruIoctl(1, 0x8004, &request, &output) == 15);
    CHECK(output == 0xAABBCCDD);
    CHECK(strstr(api_log, "interval=30 condition=1") != NULL);
    CHECK(strstr(api_log, "role=tx protocol=3 txflags=0x00000200") != NULL);
    CHECK(strstr(api_log, "declared=300 captured=300") != NULL);
    CHECK(strstr(api_log, "role=tx offset=288 bytes=12: 20 21 22 23 24 25 26 27 28 29 2A 2B") != NULL);
    CHECK(strstr(api_log, "role=mask offset=0 bytes=2: 00 FF") != NULL);
    CHECK(strstr(api_log, "role=pattern offset=0 bytes=2: 00 78") != NULL);
    CHECK(PassThruIoctl(1, 0x8005, &message_id, &output) == 15);
    CHECK(output == 0xAABBCCDD);
    CHECK(PassThruIoctl(1, 0x8006, &message_id, (void *)1) == 15);
    CHECK(strstr(api_log, "operation=QUERY") && strstr(api_log, "operation=STOP"));
    CHECK(strstr(api_log, "message_id=42") != NULL);
    CHECK(PassThruIoctl(1, 0x8004, (void *)1, NULL) == 15);
    CHECK(PassThruIoctl(1, 0x8005, NULL, NULL) == 15);
    CHECK(strstr(api_log, "descriptor=unreadable") != NULL);
    CHECK(strstr(api_log, "message_id=unreadable") != NULL);
    request.tx.DataSize = 0xFFFFFFFF;
    CHECK(PassThruIoctl(1, 0x8004, &request, NULL) == 15);
    CHECK(strstr(api_log, "declared=4294967295 captured=4128") != NULL);
    CHECK(controls == 0 && writes == 0 && inits == 0);
    capture_logging = false;
}

typedef struct { long rc; unsigned long count; PASSTHRU_MSG msg; } call_result;
static DWORD WINAPI read_call(LPVOID result)
{
    call_result *r = result; r->count = 1;
    r->rc = PassThruReadMsgs(1, &r->msg, &r->count, 1000);
    return 0;
}
static DWORD WINAPI open_call(LPVOID result)
{
    call_result *r = result;
    r->rc = PassThruOpen(NULL, &r->count); return 0;
}
static DWORD WINAPI connect_call(LPVOID result)
{
    call_result *r = result;
    r->rc = PassThruConnect(1, J2534_CAN, 0, 500000, &r->count); return 0;
}
static DWORD WINAPI close_call(LPVOID result)
{
    call_result *r = result;
    r->rc = PassThruClose(1); SetEvent(close_done); return 0;
}
static void join(HANDLE thread)
{
    if (WaitForSingleObject(thread, 3000) != WAIT_OBJECT_0) {
        puts("FAIL: test thread hung"); ExitProcess(1);
    }
    CloseHandle(thread);
}
static void barriers(void)
{
    ResetEvent(entered); ResetEvent(release_call); ResetEvent(entered_twice); ResetEvent(close_done);
}

static int repeat_sends;
static long repeat_send_error;
static bool repeat_block;
static uint8_t repeat_last[4142];
static uint16_t repeat_last_len;
static long repeat_test_send(const uint8_t *data, uint16_t len)
{
    repeat_sends++; repeat_last_len = len; memcpy(repeat_last, data, len);
    if (repeat_block) {
        SetEvent(entered);
        CHECK(WaitForSingleObject(release_call, 2000) == WAIT_OBJECT_0);
    }
    return repeat_send_error;
}
static void repeat_reply(const uint8_t *data, unsigned n, uint32_t status)
{
    uint8_t frame[64] = {0x80, 0, 0, 0};
    frame[4] = (uint8_t)(status >> 24); frame[5] = (uint8_t)(status >> 16);
    frame[6] = (uint8_t)(status >> 8); frame[7] = (uint8_t)status;
    frame[9] = (uint8_t)n; memcpy(frame + 10, data, n);
    route_rx(0, frame, 10 + (int)n, 0);
}
/* Honda K-line reply [hdr][len][data][cs] whose byte at position n-1 is v and
 * whose total length is n+1 -- 01 04 v cs for ABS (n=3), 07 05 01 v cs for
 * TPMS (n=4), the shapes seen live on 2026-09-28. */
static void honda_reply(unsigned n, uint8_t v, uint32_t status)
{
    uint8_t f[16] = {0};
    f[0] = n == 3 ? 0x01 : 0x07; f[1] = (uint8_t)(n + 1);
    for (unsigned i = 2; i < n - 1; ++i) f[i] = 0x01;
    f[n - 1] = v;
    uint8_t sum = 0;
    for (unsigned i = 0; i < n; ++i) sum = (uint8_t)(sum + f[i]);
    f[n] = (uint8_t)(0 - sum);
    repeat_reply(f, n + 1, status);
}
/* Advance well past interval, P3 and the reply timeout, then run the scheduler. */
static void repeat_due(void) { repeat_test_now += 1000; repeat_tick(repeat_test_now); }
static void repeat_at(DWORD ms) { repeat_test_now += ms; repeat_tick(repeat_test_now); }
static DWORD WINAPI repeat_tick_call(void *unused)
{
    (void)unused; repeat_tick(repeat_test_now); return 0;
}
static DWORD WINAPI repeat_stop_call(void *result)
{
    call_result *r = result;
    r->rc = PassThruIoctl(1, 0x8006, &r->count, NULL); SetEvent(close_done); return 0;
}
static void repeat_behavior_test(void)
{
    static const uint8_t tpms_tx[] = {0xA7,5,1,1,0x52}, abs_tx[] = {0xA1,4,1,0x5A};
    for (unsigned variant = 0; variant < 2; ++variant) {
        reset(J2534_ISO9141); repeat_sends = 0; repeat_send_error = 0;
        repeat_log_input input = {0};
        input.interval = 30; input.condition = 1;
        input.tx.ProtocolID = input.mask.ProtocolID = input.pattern.ProtocolID = 3;
        input.tx.TxFlags = input.mask.TxFlags = input.pattern.TxFlags = 0x200;
        input.tx.RxStatus = 0x046C9320; input.tx.ExtraDataIndex = 74224566; /* observed unused fields */
        input.tx.DataSize = variant ? sizeof(abs_tx) : sizeof(tpms_tx);
        memcpy(input.tx.Data, variant ? abs_tx : tpms_tx, input.tx.DataSize);
        unsigned n = variant ? 3 : 4;
        input.mask.DataSize = input.pattern.DataSize = n;
        input.mask.Data[n-1] = 255; input.pattern.Data[n-1] = 0x31;
        unsigned long id = 0, status = 0xAABBCCDD, stale;
        CHECK(PassThruIoctl(1, 0x8004, &input, (void *)1) == ERR_NULL_PARAMETER);
        CHECK(!s_repeat[0].used && repeat_sends == 0);
        CHECK(PassThruIoctl(1, 0x8004, &input, &id) == 0 && id != 0);
        CHECK(s_repeat[0].honda_framed);
        CHECK(PassThruIoctl(1, 0x8004, &input, &status) == ERR_EXCEEDED_LIMIT);
        CHECK(status == 0xAABBCCDD);
        honda_reply(n, 0x30, 0); /* pre-send traffic cannot complete */
        CHECK(PassThruIoctl(1, 0x8005, &id, &status) == 0 && status == 1);
        repeat_due(); CHECK(repeat_sends == 1 && s_repeat[0].awaiting);
        CHECK(repeat_last_len == 14 + input.tx.DataSize);
        CHECK(!memcmp(repeat_last + 14, input.tx.Data, input.tx.DataSize));
        CHECK(rd32le(repeat_last + 8) == 0x200);
        honda_reply(n, 0x30, J2534_RX_TX_MSG_TYPE); /* echo: not the ECU's turn */
        CHECK(s_repeat[0].awaiting);
        static const uint8_t partial[2] = {0x01, 0x31};
        repeat_reply(partial, 2, 0); /* incomplete condition, but it ends the turn */
        CHECK(s_repeat[0].active && !s_repeat[0].awaiting);
        honda_reply(n, 0x31, 0); repeat_due();
        CHECK(repeat_sends == 2 && s_repeat[0].active);
        int queued = s_channels[0].count;
        honda_reply(n, 0x30, 0);
        CHECK(s_channels[0].count == queued + 1); /* matcher never consumes reply */
        CHECK(PassThruIoctl(1, 0x8005, &id, &status) == 0 && status == 0);
        honda_reply(n, 0x31, 0); repeat_due();
        CHECK(repeat_sends == 2); /* latched stop */
        CHECK(PassThruIoctl(1, 0x8006, &id, NULL) == 0);
        CHECK(PassThruIoctl(1, 0x8005, &id, &status) == ERR_INVALID_MSG_ID);
        stale = id;
        CHECK(PassThruIoctl(1, 0x8004, &input, &id) == 0 && id != stale);
        CHECK(PassThruIoctl(1, 0x8006, &stale, NULL) == ERR_INVALID_MSG_ID && s_repeat[0].active);
        repeat_send_error = -1; repeat_due();
        CHECK(PassThruIoctl(1, 0x8005, &id, &status) == ERR_DEVICE_NOT_CONNECTED);
        CHECK(!s_repeat[0].active && !s_repeat_awaiting[0]); repeat_send_error = 0;
        CHECK(PassThruIoctl(1, 0x8006, &id, NULL) == 0);
        input.condition = 0;
        CHECK(PassThruIoctl(1, 0x8004, &input, &id) == 0);
        repeat_due(); honda_reply(n, 0x31, 0); CHECK(!s_repeat[0].active);
        dev_channel_remove(1); dev_channel_add(1, 3);
        CHECK(PassThruIoctl(1, 0x8005, &id, &status) == ERR_INVALID_MSG_ID);
        input.condition = 1;
        CHECK(PassThruIoctl(1, 0x8004, &input, &id) == 0);
        repeat_test_now += 1000; barriers(); repeat_block = true;
        HANDLE sender = CreateThread(NULL, 0, repeat_tick_call, NULL, 0, NULL);
        CHECK(WaitForSingleObject(entered, 1000) == WAIT_OBJECT_0);
        call_result stop = {0}; stop.count = id;
        HANDLE stopper = CreateThread(NULL, 0, repeat_stop_call, &stop, 0, NULL);
        CHECK(WaitForSingleObject(close_done, 30) == WAIT_TIMEOUT); /* waits for in-flight write */
        SetEvent(release_call); join(sender); join(stopper); repeat_block = false;
        CHECK(stop.rc == 0); int sends = repeat_sends; repeat_due(); CHECK(sends == repeat_sends);
        CHECK(!s_repeat_awaiting[0]);
        CHECK(PassThruIoctl(1, 0x8004, &input, &id) == 0);
        periodic_stop_all(); repeat_due(); CHECK(sends == repeat_sends);
        input.condition = 2; CHECK(PassThruIoctl(1, 0x8004, &input, &id) == ERR_INVALID_IOCTL_VALUE);
        input.condition = 1; input.interval = 0;
        CHECK(PassThruIoctl(1, 0x8004, &input, &id) == ERR_INVALID_IOCTL_VALUE);
        input.interval = 30;
        CHECK(PassThruIoctl(1, 0x8004, &input, &id) == 0);
        InterlockedExchange(&s_link_dead, 1); repeat_due();
        CHECK(PassThruIoctl(1, 0x8005, &id, &status) == ERR_DEVICE_NOT_CONNECTED);
        InterlockedExchange(&s_link_dead, 0); dev_channels_clear();
    }
    /* Exercise the production worker startup/join path, with only serial TX
     * replaced. Close must wait for an in-flight write and leave no worker. */
    DWORD saved_now = repeat_test_now; repeat_test_now = 0; /* worker uses real time */
    reset(J2534_ISO9141); barriers(); repeat_test_manual = false; repeat_block = true;
    repeat_log_input input = {0}; input.interval = 30; input.condition = 1;
    input.tx.ProtocolID = input.mask.ProtocolID = input.pattern.ProtocolID = 3;
    input.tx.DataSize = 4; memcpy(input.tx.Data, abs_tx, 4);
    input.mask.DataSize = input.pattern.DataSize = 3;
    input.mask.Data[2] = 255; input.pattern.Data[2] = 0x31;
    unsigned long id;
    CHECK(PassThruIoctl(1, 0x8004, &input, &id) == 0);
    CHECK(WaitForSingleObject(entered, 1000) == WAIT_OBJECT_0);
    call_result closed = {0}; HANDLE closer = CreateThread(NULL, 0, close_call, &closed, 0, NULL);
    CHECK(WaitForSingleObject(close_done, 30) == WAIT_TIMEOUT);
    SetEvent(release_call); join(closer);
    CHECK(closed.rc == 0 && s_periodic_thread == NULL && !s_repeat[0].used);
    repeat_block = false; repeat_test_manual = true; repeat_test_now = saved_now;
}

/* K-line is half-duplex: the scheduler must take turns with the ECU.  Timings
 * are the ABS clear from the 2026-09-28 session (case C5): interval 30 ms,
 * reply ~77 ms after the request. */
static void repeat_halfduplex_test(void)
{
    static const uint8_t abs_tx[] = {0xA1,4,1,0x5A};
    reset(J2534_ISO9141); repeat_sends = 0; repeat_send_error = 0;
    cfg_set(0, J2534_CFG_P3_MIN, 10);
    repeat_log_input input = {0}; input.interval = 30; input.condition = 1;
    input.tx.ProtocolID = input.mask.ProtocolID = input.pattern.ProtocolID = 3;
    input.tx.TxFlags = 0x200;
    input.tx.DataSize = sizeof(abs_tx); memcpy(input.tx.Data, abs_tx, sizeof(abs_tx));
    input.mask.DataSize = input.pattern.DataSize = 3;
    input.mask.Data[2] = 255; input.pattern.Data[2] = 0x31;
    unsigned long id = 0, active = 0;
    CHECK(PassThruIoctl(1, 0x8004, &input, &id) == 0);
    CHECK(s_repeat[0].p3_ms == 10 && s_repeat[0].honda_framed);
    repeat_at(10); CHECK(repeat_sends == 1 && s_repeat[0].awaiting);

    /* 30 ms timer points pass while the ECU is still answering: B001 sent here. */
    for (int t = 0; t < 75; ++t) repeat_at(1);
    CHECK(repeat_sends == 1);
    honda_reply(3, 0x31, 0);                /* 01 04 31 CA: clear in progress */
    CHECK(!s_repeat[0].awaiting && s_repeat[0].active);
    repeat_at(9); CHECK(repeat_sends == 1); /* inside P3_MIN */
    repeat_at(1); CHECK(repeat_sends == 2); /* P3_MIN over, interval long past */

    /* Collision debris from the live log ends the ECU's turn but never completes. */
    static const uint8_t debris[][3] = {{0x04,0x31,0xCA},{0x00,0x31,0xCA},{0xF0,0x11,0xEA}};
    for (unsigned k = 0; k < 3; ++k) {
        LONG ignored = s_stat.repeat_malformed;
        repeat_reply(debris[k], 3, 0);
        CHECK(s_repeat[0].active && !s_repeat[0].awaiting);
        CHECK(s_stat.repeat_malformed == ignored + 1);
        repeat_at(30); CHECK(repeat_sends == 3 + (int)k);
    }
    static const uint8_t two[] = {0x31,0xCA};
    repeat_reply(two, 2, 0); CHECK(s_repeat[0].active && !s_repeat[0].awaiting);
    repeat_at(30); CHECK(repeat_sends == 6);

    /* Silence: resend only after the reply timeout, never on the 30 ms timer. */
    LONG no_reply = s_stat.repeat_no_reply;
    repeat_at(s_repeat_reply_timeout_ms - 1); CHECK(repeat_sends == 6);
    repeat_at(1); CHECK(repeat_sends == 6 && !s_repeat_awaiting[0]); /* wire freed a pass */
    repeat_at(1); CHECK(repeat_sends == 7 && s_repeat[0].awaiting);
    CHECK(s_stat.repeat_no_reply == no_reply + 1);

    /* A BLOCK-filtered frame (HDS's keep-alive reply) still ends the turn. */
    s_block[0][0].in_use = true; s_block[0][0].len = 4;
    memset(s_block[0][0].mask, 0xFF, 4); memcpy(s_block[0][0].patt, "\x00\x04\x06\xF6", 4);
    int queued = s_channels[0].count;
    repeat_reply((const uint8_t *)"\x00\x04\x06\xF6", 4, 0);
    CHECK(!s_repeat[0].awaiting && s_repeat[0].active && s_channels[0].count == queued);
    memset(s_block, 0, sizeof(s_block));
    repeat_at(30); CHECK(repeat_sends == 8 && s_repeat[0].awaiting);

    /* The periodic scheduler sees the outstanding reply and holds its sends. */
    CHECK(s_repeat_awaiting[0]);
    honda_reply(3, 0x31, 0); CHECK(!s_repeat_awaiting[0]);

    /* Valid terminating reply completes and latches. */
    repeat_at(30); CHECK(repeat_sends == 9);
    honda_reply(3, 0x00, 0);
    CHECK(PassThruIoctl(1, 0x8005, &id, &active) == 0 && active == 0);
    CHECK(!s_repeat_awaiting[0]);
    repeat_at(1000); CHECK(repeat_sends == 9);

    /* STOP while a reply is outstanding sends nothing more and frees the gate. */
    CHECK(PassThruIoctl(1, 0x8006, &id, NULL) == 0);
    CHECK(PassThruIoctl(1, 0x8004, &input, &id) == 0);
    repeat_at(10); CHECK(repeat_sends == 10 && s_repeat_awaiting[0]);
    CHECK(PassThruIoctl(1, 0x8006, &id, NULL) == 0);
    CHECK(!s_repeat_awaiting[0]);
    repeat_at(1000); CHECK(repeat_sends == 10);

    /* A request without Honda framing keeps the plain mask/pattern semantics. */
    input.tx.TxFlags = 0;
    CHECK(PassThruIoctl(1, 0x8004, &input, &id) == 0 && !s_repeat[0].honda_framed);
    repeat_at(10); repeat_reply(debris[2], 3, 0);   /* byte 3 = 0xEA != 0x31 */
    CHECK(PassThruIoctl(1, 0x8005, &id, &active) == 0 && active == 0);
    CHECK(PassThruIoctl(1, 0x8006, &id, NULL) == 0);

    CHECK(PassThruIoctl(99, 0x8005, &id, &active) == ERR_INVALID_CHANNEL_ID);
    dev_channels_clear();
}

#ifdef VCX_RESEARCH_CONFIG
static void repeat_repin_test(void)
{
    /* Research pin overrides must obey the same busy-channel contract as
     * ordinary periodic messages, including completed records awaiting STOP. */
    ini_option_t *pin_option = NULL;
    for (unsigned i = 0; i < sizeof(s_ini)/sizeof(s_ini[0]); i++)
        if (!strcmp(s_ini[i].key, "kline_pin")) pin_option = &s_ini[i];
    CHECK(pin_option != NULL);
    if (!pin_option) return;
    char saved[MAX_PATH]; memcpy(saved, pin_option->value, sizeof(saved));
    strcpy(pin_option->value, "any");
    reset(J2534_ISO9141); s_uart_pin[0] = 7;
    cfg_set(0, J2534_2_CFG_J1962_PINS, 0x0700);
    cfg_set(0, J2534_CFG_P3_MIN, 25);
    repeat_log_input input = {0}; input.interval = 30; input.condition = 1;
    input.tx.ProtocolID = input.mask.ProtocolID = input.pattern.ProtocolID = 3;
    input.tx.DataSize = 4; memcpy(input.tx.Data, "\xA1\x04\x01\x5A", 4);
    input.mask.DataSize = input.pattern.DataSize = 3;
    input.mask.Data[2] = 255; input.pattern.Data[2] = 0x31;
    unsigned long id = 0, active = 0;
    CHECK(PassThruIoctl(1, 0x8004, &input, &id) == 0);
    SCONFIG params[] = {{J2534_CFG_P3_MIN, 99}, {J2534_2_CFG_J1962_PINS, 0x0F00}};
    SCONFIG_LIST config = {2, params};
    for (unsigned completed = 0; completed < 2; completed++) {
        if (completed) { repeat_due(); honda_reply(3, 0x30, 0); }
        CHECK(PassThruIoctl(1, 0x8005, &id, &active) == 0 && active == !completed);
        CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &config, NULL) == ERR_CHANNEL_IN_USE);
        CHECK(controls == 0 && s_uart_pin[0] == 7);
        CHECK(cfg_get(0, J2534_2_CFG_J1962_PINS) == 0x0700);
        CHECK(cfg_get(0, J2534_CFG_P3_MIN) == 25);
        CHECK(PassThruIoctl(1, 0x8005, &id, &active) == 0 && active == !completed);
    }
    CHECK(PassThruIoctl(1, 0x8006, &id, NULL) == 0);
    CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &config, NULL) == 0);
    CHECK(controls > 0 && s_uart_pin[0] == 15 && !s_repeat[0].used);
    CHECK(cfg_get(0, J2534_2_CFG_J1962_PINS) == 0x0F00);
    CHECK(cfg_get(0, J2534_CFG_P3_MIN) == 99);
    memcpy(pin_option->value, saved, sizeof(saved));
}
#endif

int main(void)
{
    dev_init(GetModuleHandle(NULL)); s_module = NULL; /* ignore installation INI */
    repeat_capture_test();
    entered = CreateEvent(NULL, TRUE, FALSE, NULL);
    release_call = CreateEvent(NULL, TRUE, FALSE, NULL);
    entered_twice = CreateEvent(NULL, TRUE, FALSE, NULL);
    close_done = CreateEvent(NULL, TRUE, FALSE, NULL);
    repeat_behavior_test();
    repeat_halfduplex_test();
#ifdef VCX_RESEARCH_CONFIG
    repeat_repin_test();
#endif
    reset(J2534_CAN);

    struct { char text[80]; unsigned char guard[160]; } error;
    memset(&error, 0xA5, sizeof(error));
    char detail[220]; memset(detail, 'x', sizeof(detail)-1); detail[sizeof(detail)-1] = 0;
    set_err(ERR_FAILED, "%s", detail);
    CHECK(PassThruGetLastError(error.text) == 0); CHECK(strlen(error.text) == 79);
    for (unsigned i = 0; i < sizeof(error.guard); i++) CHECK(error.guard[i] == 0xA5);

    reset(J2534_ISO9141);
    uint8_t address = 0x33;
    struct { uint8_t key[1], guard[3]; } short_keys = {{0}, {0xA5,0xA5,0xA5}};
    SBYTE_ARRAY input = {1, &address}, output = {1, short_keys.key};
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &input, &output) == ERR_INVALID_IOCTL_VALUE);
    CHECK(inits == 0 && short_keys.guard[0] == 0xA5);
    uint8_t keys[2]; output.BytePtr = keys; output.NumOfBytes = 0;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &input, &output) == ERR_INVALID_IOCTL_VALUE);
    output.NumOfBytes = 2; input.NumOfBytes = 2;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &input, &output) == ERR_INVALID_IOCTL_VALUE);
    input.NumOfBytes = 1;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &input, &output) == 0);
    CHECK(output.NumOfBytes == 2 && keys[0] == 8 && keys[1] == 0x94);
    CHECK(PassThruIoctl(1, J2534_IOCTL_FAST_INIT, NULL, NULL) == 0);
    CHECK(last_init_len == 0);
    PASSTHRU_MSG msg = {0}; msg.ProtocolID = J2534_ISO9141; msg.DataSize = 3;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FAST_INIT, &msg, NULL) == 0); CHECK(last_init_len == 3);

    reset(J2534_2_UART_ECHO_BYTE_PS);
    api_log_used = 0; api_log[0] = 0; capture_logging = true;
    keys[0] = keys[1] = 0; output.NumOfBytes = 2;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &input, &output) == 0);
    CHECK(inits == 1 && output.NumOfBytes == 2 && keys[0] == 8 && keys[1] == 0x94);
    CHECK(strstr(api_log, "key bytes 08 94, measured sync baud 10400") != NULL);
    capture_logging = false;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FAST_INIT, NULL, NULL) == ERR_NOT_SUPPORTED);
    CHECK(inits == 1);

    /* HDS's EPS timing (09-28 live log): firmware worst case 300 idle + 2000
     * address + 1000 + 200 + 120 + 25 W4min + 50 W4max + 10 ms. No idle push. */
    uint32_t worst = 0;
    cfg_set(0, J2534_2_CFG_UEB_T1_MAX, 1000); cfg_set(0, J2534_2_CFG_UEB_T2_MAX, 200);
    cfg_set(0, J2534_2_CFG_UEB_T3_MAX, 120);
    xact_n = 0;
    CHECK(dev_five_baud_begin(1, &worst) == 0); CHECK(worst == 3705); CHECK(xact_n == 0);
    dev_five_baud_end(1, false); CHECK(xact_n == 0);

    /* W5 700 on ISO14230 (HDS): applied as the idle for this init, then the
     * 300 ms fast-init idle is put back. */
    reset(J2534_ISO14230);
    cfg_set(0, J2534_CFG_W5, 700);
    xact_n = 0; keys[0] = keys[1] = 0; output.NumOfBytes = 2;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &input, &output) == 0);
    CHECK(output.NumOfBytes == 2 && keys[0] == 8 && keys[1] == 0x94);
    const uint8_t idle700[] = {0,0x45,0,0x0A,0xAE,0x60}, idle300[] = {0,0x45,0,0x04,0x93,0xE0};
    CHECK(xact_n == 2 && xacts[0].op == VCX_OP_PARAMS && xacts[1].op == VCX_OP_PARAMS);
    CHECK(!memcmp(xacts[0].data, idle700, 6) && !memcmp(xacts[1].data, idle300, 6));
    CHECK(dev_five_baud_begin(1, &worst) == 0 && worst == 3375);
    dev_five_baud_end(1, false);

    /* A bogus sync measurement is a failure, not two trustworthy key bytes. */
    reset(J2534_ISO9141);
    init_baud = 100; keys[0] = keys[1] = 0xAA; output.NumOfBytes = 2;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &input, &output) == ERR_FAILED);
    CHECK(output.NumOfBytes == 0 && keys[0] == 0xAA);
    init_baud = 10400;

    /* A failed init re-enables firmware periodics on every channel using them
     * (a single sync edge leaves every firmware timer paused). */
    reset(J2534_ISO9141);
    s_fw_slot_used[3] = 1; init_silent = true; xact_n = 0; output.NumOfBytes = 2;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &input, &output) == ERR_TIMEOUT);
    CHECK(xact_n == 1 && xacts[0].op == VCX_OP_PERIODIC_CTRL && xacts[0].ch == 3 &&
          xacts[0].len == 1 && xacts[0].data[0] == 1);
    init_silent = false; s_fw_slot_used[3] = 0;

    reset(J2534_CAN);
    PASSTHRU_MSG mask = {0}, pattern = {0}, flow = {0}; unsigned long id = 0xDEAD;
    mask.ProtocolID = pattern.ProtocolID = flow.ProtocolID = J2534_CAN;
    mask.DataSize = pattern.DataSize = flow.DataSize = 4;
    memset(mask.Data, 0xFF, 4);
    flow.DataSize = sizeof(flow.Data);
    CHECK(PassThruStartMsgFilter(1, J2534_PASS_FILTER, &mask, &pattern, &flow, &id) == 0);
    CHECK(filter_flow_len == 0); CHECK(PassThruStopMsgFilter(1, id) == 0);
    CHECK(PassThruStartMsgFilter(1, J2534_BLOCK_FILTER, &mask, &pattern, (PASSTHRU_MSG *)(uintptr_t)1, &id) == 0);
    CHECK(filter_flow_len == 0); CHECK(PassThruStopMsgFilter(1, id) == 0);
    int before = controls; pattern.ProtocolID = J2534_ISO9141;
    CHECK(PassThruStartMsgFilter(1, J2534_PASS_FILTER, &mask, &pattern, NULL, &id) == ERR_MSG_PROTOCOL_ID);
    pattern.ProtocolID = J2534_CAN; mask.ProtocolID = J2534_ISO9141;
    CHECK(PassThruStartMsgFilter(1, J2534_PASS_FILTER, &mask, &pattern, NULL, &id) == ERR_MSG_PROTOCOL_ID);
    mask.ProtocolID = J2534_CAN; flow.ProtocolID = J2534_ISO9141; flow.DataSize = 4;
    CHECK(PassThruStartMsgFilter(1, J2534_FLOW_CONTROL_FILTER, &mask, &pattern, &flow, &id) == ERR_MSG_PROTOCOL_ID);
    CHECK(PassThruStartPeriodicMsg(1, &msg, &id, 100) == ERR_MSG_PROTOCOL_ID);
    CHECK(controls == before);

    /* HDS SRS keepalive: HONDA_DIAGH_PS message, interval
     * 3000, on an ISO9141 channel driving pin 7.  Accepted only in
     * vendor-compatible mode, only for that pair and pin. */
    {
        PASSTHRU_MSG ka = {0}; unsigned long kid = 0xDEAD;
        static const uint8_t ka_data[] = {0x60, 0x05, 0x70, 0x02, 0x29};
        ka.ProtocolID = J2534_2_HONDA_DIAGH_PS; ka.TxFlags = 0x200;
        ka.DataSize = sizeof(ka_data); memcpy(ka.Data, ka_data, sizeof(ka_data));
        reset(J2534_ISO9141); s_uart_pin[0] = 7;
        CHECK(PassThruStartPeriodicMsg(1, &ka, &kid, 3000) == STATUS_NOERROR);
        CHECK(kid != 0xDEAD);
        reset(J2534_ISO9141); s_uart_pin[0] = 7; s_strict_validation = true; kid = 0xDEAD;
        CHECK(PassThruStartPeriodicMsg(1, &ka, &kid, 3000) == ERR_MSG_PROTOCOL_ID);
        reset(J2534_ISO9141); s_uart_pin[0] = 1; before = controls;
        CHECK(PassThruStartPeriodicMsg(1, &ka, &kid, 3000) == ERR_MSG_PROTOCOL_ID);
        reset(J2534_CAN); s_uart_pin[0] = 7;
        CHECK(PassThruStartPeriodicMsg(1, &ka, &kid, 3000) == ERR_MSG_PROTOCOL_ID);
        reset(J2534_ISO9141); s_uart_pin[0] = 7; ka.ProtocolID = J2534_2_ISO9141_PS;
        CHECK(PassThruStartPeriodicMsg(1, &ka, &kid, 3000) == ERR_MSG_PROTOCOL_ID);
        CHECK(kid == 0xDEAD);
        s_uart_pin[0] = 0; reset(J2534_CAN); before = controls;
    }

    unsigned long value = 0xDEADBEEF;
    CHECK(PassThruIoctl(99, J2534_IOCTL_READ_VBATT, NULL, &value) == ERR_INVALID_DEVICE_ID);
    CHECK(PassThruIoctl(99, J2534_IOCTL_CLEAR_FUNCT_MSG_LOOKUP_TABLE, NULL, NULL) == ERR_INVALID_CHANNEL_ID);
    CHECK(PassThruIoctl(1, J2534_IOCTL_CLEAR_FUNCT_MSG_LOOKUP_TABLE, NULL, NULL) == ERR_NOT_SUPPORTED);
    g_open = 0;
    CHECK(PassThruIoctl(1, J2534_IOCTL_READ_PROG_VOLTAGE, NULL, &value) == ERR_INVALID_DEVICE_ID);
    CHECK(controls == before && value == 0xDEADBEEF); g_open = 1;

    short_type = PT_CMD_START_FILTER; id = 0xDEAD;
    CHECK(PassThruStartMsgFilter(1, J2534_PASS_FILTER, &mask, &pattern, NULL, &id) == ERR_FAILED);
    CHECK(id == 0xDEAD);
    short_type = PT_CMD_START_PERIODIC; msg.ProtocolID = J2534_CAN; msg.DataSize = 5;
    CHECK(PassThruStartPeriodicMsg(1, &msg, &id, 100) == ERR_FAILED); CHECK(id == 0xDEAD);
    short_type = PT_CMD_IOCTL;
    CHECK(PassThruIoctl(1, J2534_IOCTL_READ_VBATT, NULL, &value) == ERR_FAILED);
    CHECK(PassThruIoctl(1, J2534_IOCTL_READ_PROG_VOLTAGE, NULL, &value) == ERR_FAILED);
    CHECK(value == 0xDEADBEEF);
    SCONFIG cfg = {J2534_CFG_DATA_RATE, 0xDEAD}; SCONFIG_LIST list = {1, &cfg};
    CHECK(PassThruIoctl(1, J2534_IOCTL_GET_CONFIG, &list, NULL) == ERR_FAILED); CHECK(cfg.Value == 0xDEAD);
    short_type = 0;
    /* Real backend must reject short raw voltage replies, too. */
    CHECK(PassThruIoctl(1, J2534_IOCTL_READ_VBATT, NULL, &value) == ERR_FAILED);
    CHECK(value == 0xDEADBEEF);

    reset(J2534_CAN);
    PASSTHRU_MSG batch[3] = {0};
    for (int i = 0; i < 3; i++) { batch[i].ProtocolID = J2534_CAN; batch[i].DataSize = 5; }
    unsigned long count = 3; delay_write = 20;
    CHECK(PassThruWriteMsgs(1, batch, &count, 5) == ERR_TIMEOUT); CHECK(count == 1 && writes == 1);
    delay_write = 0; writes = 0; count = 3;
    CHECK(PassThruWriteMsgs(1, batch, &count, 0) == 0); CHECK(count == 3 && writes == 3);
    push(0); count = 3;
    CHECK(PassThruReadMsgs(1, batch, &count, 0) == 0); CHECK(count == 1);
    count = 1; CHECK(PassThruReadMsgs(1, batch, &count, 2) == ERR_BUFFER_EMPTY); CHECK(count == 0);
    push(0); count = 2;
    CHECK(PassThruReadMsgs(1, batch, &count, 2) == ERR_TIMEOUT); CHECK(count == 1);
    s_channels[0].overflowed = true; count = 1;
    CHECK(PassThruReadMsgs(1, batch, &count, 0) == ERR_BUFFER_OVERFLOW);
    CHECK(count == 0 && !s_channels[0].overflowed);

    push(J2534_RX_TX_MSG_TYPE); s_channels[0].tail->timestamp = 1234;
    push(J2534_RX_TX_MSG_TYPE | J2534_RX_TX_INDICATION); s_channels[0].tail->timestamp = 0xFFFFFFFFu;
    push(J2534_RX_START_OF_MESSAGE);
    count = 3;
    CHECK(PassThruReadMsgs(1, batch, &count, 0) == 0 && count == 3);
    CHECK(batch[0].Timestamp == 1234 && batch[0].ExtraDataIndex == batch[0].DataSize);
    CHECK(batch[1].Timestamp == 0xFFFFFFFFu && batch[1].ExtraDataIndex == 0);
    CHECK(batch[2].ExtraDataIndex == 0);

    /* Read waiting on an old generation must not consume the replacement. */
    reset(J2534_ISO9141); barriers(); block_read = true;
    call_result a = {0}, b = {0};
    HANDLE ta = CreateThread(NULL, 0, read_call, &a, 0, NULL);
    CHECK(WaitForSingleObject(entered, 1000) == WAIT_OBJECT_0);
    dev_channel_remove(1); dev_channel_add(1, J2534_CAN); push(0);
    s_channels[0].overflowed = true;
    SetEvent(release_call); join(ta);
    CHECK(a.rc == ERR_INVALID_CHANNEL_ID && a.count == 0);
    CHECK(s_channels[0].count == 1 && s_channels[0].overflowed);

    /* Two Open calls cannot both acquire the one device session. */
    reset(J2534_CAN); dev_channels_clear(); g_open = 0; barriers(); block_open = true;
    ta = CreateThread(NULL, 0, open_call, &a, 0, NULL);
    CHECK(WaitForSingleObject(entered, 1000) == WAIT_OBJECT_0);
    HANDLE tb = CreateThread(NULL, 0, open_call, &b, 0, NULL);
    CHECK(WaitForSingleObject(entered_twice, 50) == WAIT_TIMEOUT);
    SetEvent(release_call); join(ta); join(tb);
    CHECK(opens == 1 && a.rc == 0 && b.rc == ERR_DEVICE_IN_USE);

    /* Close cannot clear a channel reservation before API registration. */
    reset(J2534_CAN); dev_channels_clear(); barriers(); block_connect = true;
    ta = CreateThread(NULL, 0, connect_call, &a, 0, NULL);
    CHECK(WaitForSingleObject(entered, 1000) == WAIT_OBJECT_0);
    tb = CreateThread(NULL, 0, close_call, &b, 0, NULL);
    CHECK(WaitForSingleObject(close_done, 50) == WAIT_TIMEOUT);
    SetEvent(release_call); join(ta); join(tb);
    CHECK(a.rc == 0 && b.rc == 0 && !g_open && !dev_channel_find(1));

    reset(J2534_CAN); dev_channels_clear(); g_open = 0; bad_open = 1;
    CHECK(PassThruOpen(NULL, &id) == ERR_FAILED);
    CHECK(!g_open && !s_session_active && s_com == INVALID_HANDLE_VALUE);
    CloseHandle(entered); CloseHandle(release_call); CloseHandle(entered_twice); CloseHandle(close_done);

    /* GETINFO block captured from a 1.9.4.2 Nano through the vendor DLL
     * (logs/VCX.raw.j2534_connect_probe.exe.log), which decodes it as
     * VCX-NANO / 1.9.4.2 / 2023-03-31. */
    static const uint8_t devinfo[DEVINFO_LEN] = {
        0x2e,0x2a,0x9c,0x10,0x00,0x00,0x15,0x58,0x22,0x97,0x41,0x07,0x45,0x03,0x00,0x00,
        0x4e,0x43,0x41,0x46,0x4d,0x49,0xab,0x27,0x12,0x00,0x00,0x00,0x56,0x43,0x58,0x2d,
        0x4e,0x41,0x4e,0x4f,0x00,0x00,0x00,0x11,0x00,0x01,0x01,0x6b,0x56,0x43,0x58,0x41,
        0x2e,0x42,0x49,0x4e,0x02,0x04,0x09,0x01,0x00,0x1f,0x03,0x35,0x00,0x00,0x00,0x00};
    char fw[80];
    format_fw_version(devinfo, sizeof(devinfo), fw, sizeof(fw));
    CHECK(strcmp(fw, "1.9.4.2 (VCX-NANO 2023-03-31)") == 0);
    format_fw_version(devinfo, DEVINFO_LEN - 1, fw, sizeof(fw));
    CHECK(strcmp(fw, "unknown") == 0);
    uint8_t served[DEVINFO_LEN]; uint16_t served_len = 0xFFFF;
    CHECK(do_version(served, &served_len, sizeof(served)) == -1 && served_len == 0);
    char vers[3][80];
    g_open = 1;
    CHECK(PassThruReadVersion(1, vers[0], vers[1], vers[2]) == 0);
    CHECK(strcmp(vers[1], DLL_VERSION_FULL) == 0 && strcmp(vers[2], "04.04") == 0);
    g_open = 0;
    char module_path[MAX_PATH], module_hash[65];
    module_identity(module_path, sizeof(module_path), module_hash);
    CHECK(strlen(module_hash) == 64 && strspn(module_hash, "0123456789abcdef") == 64);

    dev_shutdown();
    printf("API regressions: %s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
