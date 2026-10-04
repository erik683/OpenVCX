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
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)
static const uint8_t slow[] = {0x55, 0x08, 0x94, 0x28, 0xA0, 0};
static const uint8_t fast[] = {0x83, 0xF1, 0x10, 0xC1, 0xE9, 0x8F};
static void push(uint32_t channel, uint32_t status, const uint8_t *bytes, uint16_t len)
{
    EnterCriticalSection(&s_chan);
    CHECK(chan_push_locked(chan_find_locked(channel), status, bytes, len) == CHAN_PUSH_OK);
    LeaveCriticalSection(&s_chan);
}
static long init_test_send(unsigned long channel, uint32_t flag, const uint8_t *data, uint16_t len)
{
    (void)data; (void)len;
    sends++;
    if (scenario == 4) return 0; /* no reply */
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
    if (flag == 8) push(channel, 0, slow, sizeof(slow));
    else {
        push(channel, 0, NULL, 0);
        push(channel, 0, slow, sizeof(slow)); /* stale slow-init metadata */
        push(channel, 0, fast, sizeof(fast));
        dev_channel_find(channel)->tail->timestamp = 1234;
    }
    return 0;
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
    CHECK(PassThruIoctl(1, J2534_IOCTL_FAST_INIT, &request, &response) == 0);
    CHECK(response.DataSize == sizeof(fast) && memcmp(response.Data, fast, sizeof(fast)) == 0);
    CHECK(response.Timestamp == 1234000);
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
