/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/* Offline regressions using the real backend/API, a scripted device and a
 * real competing RX thread. No COM port is opened. */
#include <windows.h>
#include <stdint.h>
static long config_test_xact(uint8_t, uint8_t, const uint8_t *, int);
static BOOL WINAPI test_write(HANDLE, LPCVOID, DWORD, LPDWORD, LPOVERLAPPED);
static DWORD WINAPI test_ini(LPCSTR, LPCSTR, LPCSTR, LPSTR, DWORD, LPCSTR);
#define VCX_CONFIG_TEST
#define WriteFile test_write
#define GetPrivateProfileStringA test_ini
#include "device_vcx.c"
#include "api.c"
#undef WriteFile
#undef GetPrivateProfileStringA

static int failures, warm = 1;
static long stop_result, add_result, enable_result, off_result, on_result;
static bool device_slots[DEV_MAX_CHANNELS][VCX_FW_PERIODIC_SLOTS], rail_on;
static int off_count, tx_fail;
static HANDLE rx_attempted, rx_thread;
static bool inject_rx;
static uint8_t periodic_wire[25], periodic_wire_chan;
static int periodic_wire_len, periodic_enables, port_writes;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)

static DWORD WINAPI test_ini(LPCSTR section, LPCSTR key, LPCSTR fallback,
                             LPSTR out, DWORD cap, LPCSTR path)
{
    (void)section; (void)path;
    const char *value = !strcmp(key, "keep_warm") ? (warm ? "1" : "0") : fallback;
    if (!cap) return 0;
    snprintf(out, cap, "%s", value);
    return (DWORD)strlen(out);
}

static long config_test_xact(uint8_t op, uint8_t ch, const uint8_t *p, int n)
{
    if (s_com == INVALID_HANDLE_VALUE) return -1;
    if (op == VCX_OP_PERIODIC_ADD) {
        CHECK(ch < DEV_MAX_CHANNELS && p[1] < VCX_FW_PERIODIC_SLOTS);
        if (n == 9) {
            if (stop_result) return stop_result; /* stop never reached device */
            device_slots[ch][p[1]] = false;
        } else {
            CHECK(n <= (int)sizeof(periodic_wire));
            if (n <= (int)sizeof(periodic_wire)) memcpy(periodic_wire, p, n);
            periodic_wire_len = n; periodic_wire_chan = ch;
            device_slots[ch][p[1]] = true;
            return add_result; /* negative means applied, but reply lost */
        }
    }
    if (op == VCX_OP_PERIODIC_CTRL) {
        CHECK(n == 1 && p[0] == 1);
        periodic_enables++;
        return enable_result;
    }
    if (op == 0x85) {
        if (p[1] == 0xFF) {
            off_count++;
            if (off_result) return off_result;
            rail_on = false;
        } else {
            rail_on = true;
            return on_result;
        }
    }
    return STATUS_NOERROR;
}

static DWORD WINAPI receive_during_write(LPVOID unused)
{
    (void)unused;
    /* Signal only after attempting the channel lock, making the contention
     * deterministic instead of depending on thread scheduling or Sleep. */
    BOOL acquired = TryEnterCriticalSection(&s_chan);
    SetEvent(rx_attempted);
    if (acquired) LeaveCriticalSection(&s_chan);
    uint8_t frame[15] = {0x80,0,0,0, 0,0,0,0, 0,5, 0,0,7,0xE8,0x62};
    route_rx(0, frame, sizeof(frame), 0);
    return 0;
}

static BOOL WINAPI test_write(HANDLE port, LPCVOID bytes, DWORD n,
                              LPDWORD written, LPOVERLAPPED overlapped)
{
    (void)port; (void)bytes; (void)overlapped;
    port_writes++;
    if (inject_rx) {
        rx_thread = CreateThread(NULL, 0, receive_during_write, NULL, 0, NULL);
        CHECK(rx_thread != NULL);
        CHECK(WaitForSingleObject(rx_attempted, 1000) == WAIT_OBJECT_0);
        /* The reader must be waiting for s_chan while WriteFile is active. */
        CHECK(WaitForSingleObject(rx_thread, 20) == WAIT_TIMEOUT);
    }
    *written = tx_fail ? 0 : n;
    if (tx_fail) SetLastError(ERROR_WRITE_FAULT);
    return !tx_fail;
}

static void reset_channel(void)
{
    dev_channels_clear();
    if (s_com != INVALID_HANDLE_VALUE) CloseHandle(s_com);
    s_com = CreateEvent(NULL, FALSE, FALSE, NULL); /* harmless stand-in handle */
    CHECK(s_com != NULL);
    memset(s_periodic, 0, sizeof(s_periodic));
    memset(s_fw_slot_used, 0, sizeof(s_fw_slot_used));
    memset(device_slots, 0, sizeof(device_slots));
    memset(s_cfg_n, 0, sizeof(s_cfg_n));
    s_prog_maybe_on = rail_on = false;
    s_prog_pin = 0; s_prog_target_mv = 0;
    stop_result = add_result = enable_result = off_result = on_result = 0;
    off_count = tx_fail = 0; inject_rx = false; warm = 1;
    periodic_wire_len = periodic_enables = 0;
    s_session_active = true; g_open = 1;
    CHECK(dev_channel_add(1, J2534_ISO15765) != NULL);
}

static long filter(uint32_t kind, uint32_t *id)
{
    uint8_t req[26] = {0}, resp[4] = {0}; uint16_t rn = 0;
    wr32le(req, 1); wr32le(req + 4, kind);
    req[12] = req[14] = 4;
    memset(req + 18, 0xFF, 4); req[24] = 7; req[25] = 0xE8;
    long rc = dev_request(PT_CMD_START_FILTER, req, sizeof(req), resp, &rn, 4);
    *id = rd32le(resp);
    return rc;
}

static long start_periodic(uint32_t *id)
{
    uint8_t req[19] = {0}, resp[4] = {0}; uint16_t rn = 0;
    wr32le(req, 1); wr32le(req + 4, 100);
    req[12] = 5; req[16] = 7; req[17] = 0xE0; req[18] = 0x3E;
    long rc = dev_request(PT_CMD_START_PERIODIC, req, sizeof(req), resp, &rn, 4);
    *id = rd32le(resp);
    return rc;
}

/* Exercise the public API through the real encoder. The firmware consumes
 * two distinct prefixes: BE32 TX flags, then BE32 CAN id inside J2534 Data.
 * Moving the id into the first prefix, or removing that prefix, is wrong. */
static void periodic_wire_contract(void)
{
    static const uint32_t protocols[] = { J2534_CAN, J2534_ISO15765 };
    static const uint8_t messages[][12] = {
        {0, 0, 7, 0, 0xA1},
        {0, 0, 7, 1, 0xB3, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE},
        {0, 0, 0, 0, 0xA5}, /* a caller-supplied zero id is also preserved */
    };
    static const unsigned sizes[] = {5, 12, 5};
    static const uint8_t intervals[][4] = {
        {0x40, 0x42, 0x0F, 0}, /* 1000 ms -> 1000000 us */
        {0x80, 0x4F, 0x12, 0}, /* 1200 ms -> 1200000 us */
        {0x88, 0x13, 0, 0},    /* 5 ms -> 5000 us */
    };
    static const unsigned periods[] = {1000, 1200, 5};
    for (unsigned proto = 0; proto < 2; proto++) {
        reset_channel();
        dev_channels_clear();
        CHECK(dev_channel_add(2, protocols[proto]) != NULL);
        unsigned long ids[3];
        for (unsigned i = 0; i < 3; i++) {
            PASSTHRU_MSG msg = {0};
            msg.ProtocolID = protocols[proto]; msg.DataSize = sizes[i];
            memcpy(msg.Data, messages[i], sizes[i]);
            CHECK(PassThruStartPeriodicMsg(2, &msg, &ids[i], periods[i]) == 0);
            CHECK(periodic_wire_chan == 1 && periodic_wire_len == 13 + (int)sizes[i]);
            CHECK(periodic_wire[0] == 1 && periodic_wire[1] == i);
            CHECK(periodic_wire[2] == 0xC9 && periodic_wire[3] == 4 + sizes[i]);
            CHECK(periodic_wire[4] == 0);
            CHECK(memcmp(periodic_wire + 5, intervals[i], 4) == 0);
            CHECK(rd32le(periodic_wire + 9) == 0); /* flags, not identifier */
            CHECK(memcmp(periodic_wire + 13, messages[i], sizes[i]) == 0);
            CHECK(periodic_enables == (int)i + 1 && s_periodic_thread == NULL);
        }
        for (unsigned i = 0; i < 3; i++) CHECK(PassThruStopPeriodicMsg(2, ids[i]) == 0);
    }
    /* Keep unsupported forms on the existing host path. */
    CHECK(!fw_periodic_flag(0x8001, J2534_TX_ISO15765_ADDR_TYPE, 5));
    CHECK(!fw_periodic_flag(0x8101, 0x100, 5)); /* CAN_29BIT_ID */
    CHECK(!fw_periodic_flag(0x8001, 0, 13));
    CHECK(!fw_periodic_flag(0x9001, 0, 5));
}

/* K-line keep-alives on the ISO9141 engine use the firmware timer record:
 * the message's own TX flags in buffer[0..3] (the same word do_write sends),
 * flag 0xC8 (kline_periodic=idle) or 0xC9 (kline_periodic=fixed, the
 * minimal-profile default).  The normal default, host, is checked below. */
static void kline_periodic_contract(void)
{
    static const uint8_t keepalive[5] = {0x60, 0x05, 0x70, 0x02, 0x29};
    static const uint32_t protocols[] = { J2534_ISO9141, J2534_2_HONDA_DIAGH_PS };
    static const kline_per_mode_t modes[] = { KLINE_PER_IDLE, KLINE_PER_FIXED };
    static const uint8_t flags[] = { 0xC8, 0xC9 };
    for (unsigned m = 0; m < 2; m++)
        for (unsigned proto = 0; proto < 2; proto++) {
            reset_channel();
            dev_channels_clear();
            s_kline_periodic = modes[m];
            CHECK(dev_channel_add(1, protocols[proto]) != NULL);
            PASSTHRU_MSG msg = {0};
            msg.ProtocolID = protocols[proto]; msg.TxFlags = J2534_TX_WAIT_P3_MIN_ONLY;
            msg.DataSize = sizeof(keepalive);
            memcpy(msg.Data, keepalive, sizeof(keepalive));
            unsigned long id = 0;
            CHECK(PassThruStartPeriodicMsg(1, &msg, &id, 3000) == 0);
            CHECK(periodic_wire_chan == 0 && periodic_wire_len == 18);
            CHECK(periodic_wire[0] == 1 && periodic_wire[1] == 0);
            CHECK(periodic_wire[2] == flags[m] && periodic_wire[3] == 9 && periodic_wire[4] == 0);
            CHECK(rd32le(periodic_wire + 5) == 3000000u);
            CHECK(periodic_wire[9] == 0 && periodic_wire[10] == 0 &&
                  periodic_wire[11] == 0x02 && periodic_wire[12] == 0); /* BE32 0x200 */
            CHECK(memcmp(periodic_wire + 13, keepalive, sizeof(keepalive)) == 0);
            CHECK(periodic_enables == 1 && s_periodic_thread == NULL);
            CHECK(s_periodic[0].in_use && s_periodic[0].fw && s_fw_slot_used[0] == 1);
            CHECK(PassThruStopPeriodicMsg(1, id) == 0);
            CHECK(!device_slots[0][0] && !s_periodic[0].in_use && !s_fw_slot_used[0]);
        }
    s_kline_periodic = KLINE_PER_IDLE;
    CHECK(fw_periodic_flag(0x9104, 0, 12) == 0xC8);
    CHECK(!fw_periodic_flag(0x9104, 0, 13));             /* record holds 16 bytes */
    CHECK(!fw_periodic_flag(0x9104, 0x04, 5));           /* fast-init bit */
    CHECK(!fw_periodic_flag(0x9104, 0x08, 5));           /* five-baud bit */
    CHECK(!fw_periodic_flag(0x9104, 0x40000, 5));        /* skip-P3_MIN bit */
    CHECK(!fw_periodic_flag(0x9004, 0, 5));              /* ISO14230 stays on host */
    CHECK(!fw_periodic_flag(0x9204, 0, 5));              /* KW1281 stays on host */
    s_kline_periodic = KLINE_PER_HOST;
    CHECK(!fw_periodic_flag(0x9104, J2534_TX_WAIT_P3_MIN_ONLY, 5));
    CHECK(fw_periodic_flag(0x8001, 0, 5) == 0xC9);       /* CAN unaffected */
    s_kline_periodic = KLINE_PER_DEFAULT;
}

/* HDS asks HONDA_DIAGH_PS channels for J1962 pin 14 or 1.  The refusal stands,
 * and nothing is sent on pin 7 for that channel until pin 7 is selected. */
static void refused_pin_contract(void)
{
    reset_channel();
    dev_channels_clear();
    CHECK(dev_channel_add(1, J2534_2_HONDA_DIAGH_PS) != NULL);
    s_uart_pin[0] = 7; s_uart_pin_refused[0] = false;
    static const uint8_t probe[5] = {0x60, 0x05, 0x70, 0x02, 0x29};
    PASSTHRU_MSG msg = {0};
    msg.ProtocolID = J2534_2_HONDA_DIAGH_PS; msg.DataSize = sizeof(probe);
    memcpy(msg.Data, probe, sizeof(probe));
    SCONFIG pin1 = { J2534_2_CFG_J1962_PINS, 0x0100 };
    SCONFIG_LIST list = { 1, &pin1 };
#ifdef VCX_MINIMAL_PROFILE
    /* Minimal forwards the pin to the engine (vendor behaviour); the firmware
     * then drives no line, so the DLL has nothing to suppress. */
    CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &list, NULL) == 0);
    CHECK(!s_uart_pin_refused[0] && s_uart_pin[0] == 1);
    dev_channels_clear();
    return;
#endif
    CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &list, NULL) == ERR_NOT_SUPPORTED);
    CHECK(s_uart_pin_refused[0] && s_uart_pin[0] == 7);
    /* A firmware keep-alive running when the refusal lands is stopped with its
     * id and slot kept, and re-added with its own flag once pin 7 is selected. */
    s_uart_pin_refused[0] = false;
    s_kline_periodic = KLINE_PER_IDLE;
    unsigned long fw_id = 0;
    CHECK(PassThruStartPeriodicMsg(1, &msg, &fw_id, 3000) == 0 && s_periodic[0].fw);
    CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &list, NULL) == ERR_NOT_SUPPORTED);
    CHECK(s_uart_pin_refused[0] && s_periodic[0].in_use && s_periodic[0].fw &&
          s_periodic[0].parked && s_fw_slot_used[0] == 1 && !device_slots[0][0] &&
          s_periodic_thread == NULL);
    SCONFIG lift = { J2534_2_CFG_J1962_PINS, 0x0700 };
    SCONFIG_LIST lift_list = { 1, &lift };
    CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &lift_list, NULL) == 0);
    CHECK(!s_uart_pin_refused[0] && s_periodic[0].fw && !s_periodic[0].parked &&
          device_slots[0][0] && periodic_wire[2] == 0xC8 && s_periodic_thread == NULL);
    /* Lost resume replies must fail SET_CONFIG and restore suppression. If
     * rollback also fails, the next refusal must retry the uncertain slot. */
    for (int phase = 0; phase < 2; phase++) {
        for (int rollback_fails = 0; rollback_fails < 2; rollback_fails++) {
            CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &list, NULL) == ERR_NOT_SUPPORTED);
            CHECK(s_periodic[0].parked && !device_slots[0][0]);
            if (phase) enable_result = -1; else add_result = -1;
            stop_result = rollback_fails ? -1 : 0;
            CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &lift_list, NULL) == ERR_TIMEOUT);
            CHECK(s_uart_pin_refused[0] && s_periodic[0].in_use &&
                  s_periodic[0].id == fw_id && s_fw_slot_used[0] == 1);
            CHECK(s_periodic[0].parked == !rollback_fails &&
                  device_slots[0][0] == (bool)rollback_fails);
            stop_result = add_result = enable_result = 0;
            CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &list, NULL) == ERR_NOT_SUPPORTED);
            CHECK(s_uart_pin_refused[0] && s_periodic[0].parked && !device_slots[0][0]);
            CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &lift_list, NULL) == 0);
            CHECK(!s_uart_pin_refused[0] && !s_periodic[0].parked && device_slots[0][0]);
        }
    }
    CHECK(PassThruStopPeriodicMsg(1, fw_id) == 0);
    CHECK(!device_slots[0][0] && !s_fw_slot_used[0]);
    CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &list, NULL) == ERR_NOT_SUPPORTED);
    CHECK(periodic_stop_all() == STATUS_NOERROR && s_periodic_thread == NULL);
    s_kline_periodic = KLINE_PER_DEFAULT;
    periodic_wire_len = 0;
    unsigned long n = 1; port_writes = 0;
    CHECK(PassThruWriteMsgs(1, &msg, &n, 100) == 0 && n == 1);
    CHECK(port_writes == 0);
    /* A keep-alive started now must not reach a firmware timer on pin 7. */
    unsigned long id = 0;
    CHECK(PassThruStartPeriodicMsg(1, &msg, &id, 3000) == 0);
    CHECK(s_periodic[0].in_use && !s_periodic[0].fw && periodic_wire_len == 0);
    CHECK(PassThruStopPeriodicMsg(1, id) == 0);
    CHECK(periodic_stop_all() == STATUS_NOERROR && s_periodic_thread == NULL);
    /* A rejected list does not lift the refusal, even with pin 7 in it. */
    SCONFIG mixed[2] = { { J2534_2_CFG_J1962_PINS, 0x0700 }, { J2534_CFG_LOOPBACK, 2 } };
    SCONFIG_LIST bad = { 2, mixed };
    CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &bad, NULL) != 0);
    CHECK(s_uart_pin_refused[0]);
    SCONFIG pin7 = { J2534_2_CFG_J1962_PINS, 0x0700 };
    list.ConfigPtr = &pin7;
    CHECK(PassThruIoctl(1, J2534_IOCTL_SET_CONFIG, &list, NULL) == 0);
    CHECK(!s_uart_pin_refused[0]);
    n = 1; port_writes = 0;
    CHECK(PassThruWriteMsgs(1, &msg, &n, 100) == 0 && n == 1);
    CHECK(port_writes == 1);
    /* Reconnecting starts clean. */
    s_uart_pin_refused[0] = true;
    dev_channels_clear();
}

/* Frames in one serial read are back-dated by the bytes after them, and
 * internal stamps stay increasing across reads, long idle periods and wrap. */
static void rx_stamp_contract(void)
{
    uint64_t last = 999000;
    CHECK(rx_byte_stamp(1000000, 24, 0, &last) == 1000000 - 249);  /* 23 bytes * 10.85 us */
    CHECK(rx_byte_stamp(1000000, 24, 23, &last) == 1000000);
    CHECK(rx_byte_stamp(1000100, 24, 0, &last) == 1000001);       /* not before the previous read */
    last = 0xFFFFFFF0u;
    CHECK(rx_byte_stamp(0x100000005ull, 1, 0, &last) == 5);
    CHECK(last == 0x100000005ull);
    /* A warm reader may stay idle past the signed or full 32-bit clock range. */
    last = 1000000;
    CHECK(rx_byte_stamp(2401000000ull, 1, 0, &last) == 2401000000u);
    CHECK(last == 2401000000ull);
    last = 1000000;
    CHECK(rx_byte_stamp(0x100000000ull + 2401000000ull, 1, 0, &last) == 2401000000u);
    CHECK(last == 0x100000000ull + 2401000000ull);
}

int main(void)
{
    dev_init(GetModuleHandle(NULL));
    rx_stamp_contract();
    periodic_wire_contract();
    kline_periodic_contract();
    refused_pin_contract();
    uint32_t id;
    for (uint32_t kind = J2534_PASS_FILTER; kind <= J2534_BLOCK_FILTER; kind++) {
        reset_channel();
        for (uint32_t i = 1; i <= 300; i++) {
            CHECK(filter(kind, &id) == 0); CHECK(id == i);
            CHECK(PassThruStopMsgFilter(1, id) == 0);
        }
    }

    /* Closing without explicit channel disconnect must not leak filters. */
    for (int keep = 0; keep <= 1; keep++) {
        reset_channel(); warm = keep;
        CHECK(filter(J2534_PASS_FILTER, &id) == 0);
        CHECK(filter(J2534_BLOCK_FILTER, &id) == 0);
        CHECK(PassThruClose(1) == 0);
        CHECK(s_filt_n[0] == 0 && !s_block[0][0].in_use);
        if (s_com == INVALID_HANDLE_VALUE) s_com = CreateEvent(NULL, FALSE, FALSE, NULL);
        uint8_t req[16] = {0}, resp[4]; uint16_t rn;
        wr32le(req, J2534_ISO15765); wr32le(req + 8, 500000);
        CHECK(dev_request(PT_CMD_CONNECT, req, sizeof(req), resp, &rn, 4) == 0);
        CHECK(dev_channel_add(1, J2534_ISO15765) != NULL);
        CHECK(filter(J2534_PASS_FILTER, &id) == 0);
        uint8_t rx[15] = {0x80,0,0,0, 0,0,0,0, 0,5, 0,0,7,0xE8,0x62};
        route_rx(0, rx, sizeof(rx), 0);
        CHECK(s_channels[0].count == 1);
    }

    /* A failed stop must be visible and retryable through Stop, CLEAR and Close. */
    reset_channel(); CHECK(start_periodic(&id) == 0);
    stop_result = -1;
    CHECK(PassThruStopPeriodicMsg(1, id) == ERR_TIMEOUT);
    CHECK(device_slots[0][0] && s_periodic[0].in_use && s_fw_slot_used[0] == 1);
    CHECK(PassThruIoctl(1, J2534_IOCTL_CLEAR_PERIODIC_MSGS, NULL, NULL) == ERR_TIMEOUT);
    CHECK(PassThruDisconnect(1) == ERR_TIMEOUT); CHECK(dev_channel_find(1) != NULL);
    CHECK(PassThruClose(1) == ERR_TIMEOUT); CHECK(g_open && s_session_active);
    CHECK(s_periodic[0].in_use);
    stop_result = 1;
    CHECK(PassThruStopPeriodicMsg(1, id) == ERR_FAILED);
    stop_result = 0;
    CHECK(PassThruStopPeriodicMsg(1, id) == 0);
    CHECK(!device_slots[0][0] && !s_periodic[0].in_use && !s_fw_slot_used[0]);
    CHECK(PassThruClose(1) == 0); CHECK(!g_open && !s_session_active);

    /* Lost ADD/ENABLE replies trigger device rollback; failed rollback stays
     * tracked even though Start returned an error and supplied no usable ID. */
    for (int phase = 0; phase < 2; phase++) {
        reset_channel();
        if (phase) enable_result = -1; else add_result = -1;
        CHECK(start_periodic(&id) == -1);
        CHECK(!device_slots[0][0] && !s_periodic[0].in_use && !s_fw_slot_used[0]);
        stop_result = -1;
        CHECK(start_periodic(&id) == -1);
        CHECK(device_slots[0][0] && s_periodic[0].in_use && s_fw_slot_used[0] == 1);
        stop_result = add_result = enable_result = 0;
        uint32_t second;
        CHECK(start_periodic(&second) == 0); CHECK(device_slots[0][1]);
        CHECK(PassThruIoctl(1, J2534_IOCTL_CLEAR_PERIODIC_MSGS, NULL, NULL) == 0);
        CHECK(!device_slots[0][0] && !device_slots[0][1] && !s_fw_slot_used[0]);
    }

    /* Voltage cleanup covers both close policies and ambiguous ON results. */
    for (int keep = 0; keep <= 1; keep++) {
        reset_channel(); warm = keep;
        CHECK(PassThruSetProgrammingVoltage(1, 13, 18000) == 0);
        CHECK(rail_on && s_prog_maybe_on);
        off_result = -1;
        CHECK(PassThruClose(1) == ERR_TIMEOUT);
        CHECK(rail_on && s_prog_maybe_on && g_open && s_session_active);
        off_result = 0;
        CHECK(PassThruClose(1) == 0);
        CHECK(off_count >= 2 && !rail_on && !s_prog_maybe_on);
    }
    reset_channel(); on_result = -1;
    CHECK(PassThruSetProgrammingVoltage(1, 13, 18000) == ERR_TIMEOUT);
    CHECK(rail_on && s_prog_maybe_on);
    CHECK(PassThruClose(1) == 0); CHECK(!rail_on && off_count == 1);

    /* A lost physical link retains uncertain device state, but must not
     * strand g_open and prevent a subsequent rediscovery/reconnect. */
    reset_channel(); CHECK(start_periodic(&id) == 0);
    CHECK(PassThruSetProgrammingVoltage(1, 13, 18000) == 0);
    stop_result = off_result = -1;
    dev_disconnect();
    CHECK(s_com == INVALID_HANDLE_VALUE && s_prog_maybe_on && s_periodic[0].in_use);
    CHECK(PassThruClose(1) == ERR_DEVICE_NOT_CONNECTED); CHECK(!g_open);
    s_com = CreateEvent(NULL, FALSE, FALSE, NULL); /* rediscovered link */
    off_result = 0;
    CHECK(release_prog_voltage() == 0); CHECK(!rail_on);
    uint8_t reconnect[16] = {0}, connected[4]; uint16_t connected_len;
    wr32le(reconnect, J2534_ISO15765); wr32le(reconnect + 8, 500000);
    CHECK(dev_request(PT_CMD_CONNECT, reconnect, sizeof(reconnect), connected, &connected_len, 4) == -1);
    CHECK(!s_channels[0].in_use && s_periodic[0].in_use);
    stop_result = 0;
    CHECK(dev_request(PT_CMD_CONNECT, reconnect, sizeof(reconnect), connected, &connected_len, 4) == 0);
    CHECK(!device_slots[0][0] && !s_periodic[0].in_use);

    /* Real thread contention: loopback -> TxDone -> ECU response. A write
     * failure must release the lock and must not synthesize confirmations. */
    for (int fail = 0; fail <= 1; fail++) {
        reset_channel(); cfg_set(0, J2534_CFG_LOOPBACK, 1);
        inject_rx = true; tx_fail = fail;
        rx_attempted = CreateEvent(NULL, FALSE, FALSE, NULL);
        PASSTHRU_MSG msg = {0}; msg.ProtocolID = J2534_ISO15765;
        msg.DataSize = 5; msg.Data[2] = 7; msg.Data[3] = 0xE0; msg.Data[4] = 0x22;
        unsigned long count = 1;
        CHECK(PassThruWriteMsgs(1, &msg, &count, 0) == (fail ? ERR_TIMEOUT : 0));
        CHECK(WaitForSingleObject(rx_thread, 1000) == WAIT_OBJECT_0);
        uint32_t expected[] = {1,9,0};
        for (int i = fail ? 2 : 0; i < 3; i++) {
            rx_msg_t *m = dev_channel_pop(dev_channel_find(1));
            CHECK(m && m->rx_status == expected[i]); free(m);
        }
        CHECK(s_channels[0].count == 0);
        CloseHandle(rx_thread); CloseHandle(rx_attempted);
    }
    inject_rx = false;
    CHECK(PassThruClose(1) == 0);
    dev_shutdown();
    printf("lifecycle regressions: %s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
