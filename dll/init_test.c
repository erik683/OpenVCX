/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/* Real API/backend with injected RX events; never opens a physical port. */
#include <stdint.h>
static long init_test_send(unsigned long channel, uint32_t flag, const uint8_t *data, uint16_t len);
#define VCX_INIT_TEST
#define PT_INIT_TIMEOUT_MS 10u
#include "device_vcx.c"
#include "api.c"

static int failures, scenario, sends;
static DWORD fast_stamp;  /* timestamp given to the valid fast-init reply */
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)
static const uint8_t slow[] = {0x55, 0x08, 0x94, 0x28, 0xA0, 0};
static const uint8_t fast[] = {0x83, 0xF1, 0x10, 0xC1, 0xE9, 0x8F};
static void push(uint32_t channel, uint32_t status, const uint8_t *bytes, uint16_t len)
{
    EnterCriticalSection(&s_chan);
    CHECK(chan_push_locked(chan_find_locked(channel), status, bytes, len, dev_host_us()) == CHAN_PUSH_OK);
    LeaveCriticalSection(&s_chan);
}
static long init_test_send(unsigned long channel, uint32_t flag, const uint8_t *data, uint16_t len)
{
    (void)data; (void)len;
    sends++;
    if (scenario == 4) return 0; /* no reply */
    if (scenario == 6 || scenario == 7) {
        /* A valid-looking frame enqueued at once: too early to answer the init. */
        push(channel, 0, fast, sizeof(fast));
        if (scenario == 6) {
            push(channel, 0, fast, sizeof(fast) - 1);
            dev_channel_find(channel)->tail->timestamp = fast_stamp;
        }
        return 0;
    }
    if (scenario == 5) {
        dev_channel_remove(channel);
        dev_channel_add(channel, J2534_ISO9141);
        push(channel, 0, slow, sizeof(slow));
        return 0;
    }
    push(channel, J2534_RX_TX_MSG_TYPE, slow, sizeof(slow));
    push(channel, J2534_RX_TX_INDICATION, slow, sizeof(slow));
    if (scenario == 1) { push(channel, 0, slow, 1); return 0; }
    push(channel, J2534_RX_RX_BREAK, slow, sizeof(slow));
    uint8_t malformed[6]; memcpy(malformed, slow, sizeof(malformed)); malformed[5] = 1;
    if (flag == 8) push(channel, 0, malformed, sizeof(malformed));
    push(channel, 0, slow, 2); /* not a complete firmware success record */
    if (flag == 8) {
        push(channel, 0, slow, sizeof(slow));
        /* Five-baud has no minimum reply age, whatever the clock's sign bit. */
        dev_channel_find(channel)->tail->timestamp = 0x80000000u;
    } else {
        push(channel, 0, NULL, 0);
        push(channel, 0, slow, sizeof(slow)); /* stale slow-init metadata */
        push(channel, 0, fast, sizeof(fast));
        dev_channel_find(channel)->tail->timestamp = fast_stamp;
    }
    return 0;
}

/* dev_fast_init_window: the derived wait and earliest reply time. */
static void fast_window_tests(void)
{
    uint32_t wait = 0, early = 0;
    CHECK(dev_fast_init_window(0, 4, &wait, &early) == ERR_INVALID_CHANNEL_ID);
    CHECK(dev_fast_init_window(DEV_MAX_CHANNELS + 1, 4, &wait, &early) == ERR_INVALID_CHANNEL_ID);
    /* Power-on defaults, no data rate cached: TIDLE 300 ms dominates the
     * pre-wait, TWUP 50 ms, 1200-baud byte time floor. */
    s_cfg_n[0] = 0;
    CHECK(dev_fast_init_window(1, 4, &wait, &early) == 0);
    CHECK(wait == 798 && early == 18);
    /* HDS's Honda K-line timing (2011 Civic ABS/TPMS): 80+210+4*(0.961+2)+
     * 2*(20+0.961)+300+50 = 693.8 ms; genuine replies took 254-322 ms. */
    cfg_set(0, J2534_CFG_DATA_RATE, 10400);
    cfg_set(0, J2534_CFG_P3_MIN, 80);
    cfg_set(0, J2534_CFG_P4_MIN, 2);
    cfg_set(0, J2534_CFG_TIDLE, 30);
    cfg_set(0, J2534_CFG_TINIL, 70);
    cfg_set(0, J2534_CFG_TWUP, 210);
    CHECK(dev_fast_init_window(1, 4, &wait, &early) == 0);
    CHECK(wait == 694 && early == 178);
    /* A longer ECU response window must extend the deadline, without changing
     * the earliest valid reply or shortening the existing 300 ms allowance. */
    cfg_set(0, J2534_CFG_P2_MAX, 1000);
    CHECK(dev_fast_init_window(1, 4, &wait, &early) == 0);
    CHECK(wait == 1394 && early == 178);
    s_fast_init_timeout_ms = 900;
    CHECK(dev_fast_init_window(1, 4, &wait, &early) == 0);
    CHECK(wait == 900 && early == 178);
    s_fast_init_timeout_ms = 0;
    cfg_set(0, J2534_CFG_P2_MAX, 50);
    CHECK(dev_fast_init_window(1, 4, &wait, &early) == 0);
    CHECK(wait == 694 && early == 178);
    /* Short timing never undercuts the vendor's 500 ms. */
    cfg_set(0, J2534_CFG_P3_MIN, 0);
    cfg_set(0, J2534_CFG_TIDLE, 0);
    cfg_set(0, J2534_CFG_TWUP, 20);
    cfg_set(0, J2534_CFG_P1_MAX, 5);
    CHECK(dev_fast_init_window(1, 1, &wait, &early) == 0);
    CHECK(wait == 500 && early == 0);
    /* A long wake-up pattern stretches the wait instead of being cut off. */
    cfg_set(0, J2534_CFG_TWUP, 5000);
    CHECK(dev_fast_init_window(1, 4, &wait, &early) == 0);
    CHECK(wait > 5000 && early == 4968);
    /* vcx_nano.ini fast_init_timeout_ms replaces the derived value. */
    s_fast_init_timeout_ms = 1500;
    CHECK(dev_fast_init_window(1, 4, &wait, &early) == 0);
    CHECK(wait == 1500 && early == 4968);
    s_fast_init_timeout_ms = 0;
    s_cfg_n[0] = 0;
}
int main(void)
{
    dev_init(GetModuleHandle(NULL));
    g_open = 1;
    dev_channel_add(1, J2534_ISO9141);
    uint8_t address = 0x33, keys[3] = {0xAA, 0xAA, 0xCC};
    SBYTE_ARRAY in = {1, &address}, out = {2, keys};
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &in, &out) == 0);
    CHECK(out.NumOfBytes == 2 && keys[0] == 8 && keys[1] == 0x94 && keys[2] == 0xCC);
    out.BytePtr = NULL; int before = sends;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &in, &out) == ERR_NULL_PARAMETER);
    CHECK(sends == before); out.BytePtr = keys;
    scenario = 1; keys[0] = keys[1] = 0xAA;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &in, &out) == ERR_INVALID_MSG);
    CHECK(out.NumOfBytes == 0 && keys[0] == 0xAA && keys[1] == 0xAA);
    scenario = 4;
    out.NumOfBytes = sizeof(keys);
    SetEvent(s_channels[0].rx_event); /* leftover signal is not a reply */
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &in, &out) == ERR_TIMEOUT);
    scenario = 0;
    PASSTHRU_MSG request = {0}, response = {0};
    request.ProtocolID = J2534_ISO9141; request.DataSize = 1; request.Data[0] = 0x81;
    fast_stamp = dev_host_us() + 1000000;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FAST_INIT, &request, &response) == 0);
    CHECK(response.DataSize == sizeof(fast) && memcmp(response.Data, fast, sizeof(fast)) == 0);
    CHECK(response.Timestamp == fast_stamp);
    /* A frame that predates the end of the wake-up pattern is skipped... */
    scenario = 6; fast_stamp = dev_host_us() + 1000000;
    memset(&response, 0, sizeof(response));
    CHECK(PassThruIoctl(1, J2534_IOCTL_FAST_INIT, &request, &response) == 0);
    CHECK(response.DataSize == sizeof(fast) - 1);
    /* ...and on its own is a timeout, not a reply or a malformed one. */
    scenario = 7;
    CHECK(PassThruIoctl(1, J2534_IOCTL_FAST_INIT, &request, &response) == ERR_TIMEOUT);
    fast_window_tests();
    scenario = 5;
    out.NumOfBytes = sizeof(keys);
    CHECK(PassThruIoctl(1, J2534_IOCTL_FIVE_BAUD_INIT, &in, &out) == ERR_INVALID_CHANNEL_ID);
    CHECK(dev_channel_find(1)->count == 1); /* did not consume replacement channel's data */

    dev_channel_add(2, J2534_CAN);
    push(2, 0, fast, sizeof(fast));
    s_channels[0].overflowed = true;
    s_filt_n[0] = 1; s_periodic[0].in_use = true;
    CHECK(PassThruIoctl(1, J2534_IOCTL_CLEAR_RX_BUFFER, NULL, NULL) == 0);
    CHECK(s_channels[0].head == NULL && s_channels[0].tail == NULL && s_channels[0].count == 0);
    CHECK(!s_channels[0].overflowed && WaitForSingleObject(s_channels[0].rx_event, 0) == WAIT_TIMEOUT);
    CHECK(s_channels[1].count == 1 && s_filt_n[0] == 1 && s_periodic[0].in_use);
    push(1, 0, fast, sizeof(fast));
    CHECK(s_channels[0].count == 1 && WaitForSingleObject(s_channels[0].rx_event, 0) == WAIT_OBJECT_0);
    CHECK(PassThruIoctl(99, J2534_IOCTL_CLEAR_RX_BUFFER, NULL, NULL) == ERR_INVALID_CHANNEL_ID);
    CHECK(PassThruIoctl(99, J2534_IOCTL_CLEAR_TX_BUFFER, NULL, NULL) == ERR_INVALID_CHANNEL_ID);
    s_strict_validation = false;
    CHECK(PassThruIoctl(1, J2534_IOCTL_CLEAR_TX_BUFFER, NULL, NULL) == 0);
    s_strict_validation = true;
    CHECK(PassThruIoctl(1, J2534_IOCTL_CLEAR_TX_BUFFER, NULL, NULL) == ERR_NOT_SUPPORTED);
    CHECK(s_channels[0].count == 1 && s_filt_n[0] == 1 && s_periodic[0].in_use);
    dev_channels_clear();
    printf("init/buffer API tests: %s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
