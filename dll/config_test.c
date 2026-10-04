/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/* Offline wire-contract tests: real backend, scripted control transport. */
#include <stdint.h>
static long config_test_xact(uint8_t op, uint8_t ch, const uint8_t *p, int n);
#define VCX_CONFIG_TEST
#include "device_vcx.c"

static int failures, calls, param_len;
static long params_result;
static uint8_t params[1024];
/* Filter-table traffic: every FILTINIT/ADDFILT payload, in wire order. */
static int filt_ops;
static uint8_t filt_op[32], filt_rec[32][VCX_FILTER_MAXLEN]; static int filt_len[32];
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)
static long config_test_xact(uint8_t op, uint8_t ch, const uint8_t *p, int n)
{
    (void)ch;
    calls++;
    if (op == VCX_OP_PARAMS) {
        param_len = n; memcpy(params, p, n); return params_result;
    }
    if ((op == VCX_OP_FILTINIT || op == VCX_OP_ADDFILT) && filt_ops < 32 &&
        n <= VCX_FILTER_MAXLEN) {
        filt_op[filt_ops] = op; filt_len[filt_ops] = n; memcpy(filt_rec[filt_ops], p, n);
        filt_ops++;
    }
    return 0;
}
/* PassThruStartMsgFilter(FLOW_CONTROL) for an 11-bit pattern/flow-control pair. */
static long fc_filter(uint16_t rx, uint16_t tx, uint32_t *fid)
{
    uint8_t req[18 + 12] = {0}, resp[4]; uint16_t rl = 0;
    wr32le(req, 1); wr32le(req + 4, J2534_FLOW_CONTROL_FILTER);
    wr32le(req + 8, J2534_TX_ISO15765_FRAME_PAD);
    req[12] = 4; req[14] = 4; req[16] = 4;
    uint8_t *mask = req + 18, *patt = mask + 4, *fc = patt + 4;
    mask[0] = mask[1] = mask[2] = mask[3] = 0xFF;
    patt[2] = (uint8_t)(rx >> 8); patt[3] = (uint8_t)rx;
    fc[2] = (uint8_t)(tx >> 8); fc[3] = (uint8_t)tx;
    long st = do_filter(req, sizeof(req), resp, &rl, sizeof(resp));
    if (fid) *fid = (st == 0 && rl == 4) ? rd32le(resp) : 0;
    return st;
}
/* An ADDFILT record [01 slot 02 02 04 patt(4) mask(4) fc(4)] for rx id. */
static bool fc_record(int i, uint8_t slot, uint16_t rx)
{
    const uint8_t *r = filt_rec[i];
    return filt_op[i] == VCX_OP_ADDFILT && filt_len[i] == 17 && r[0] == 0x01 &&
           r[1] == slot && r[2] == 0x02 && r[3] == 0x02 && r[4] == 0x04 &&
           r[7] == (uint8_t)(rx >> 8) && r[8] == (uint8_t)rx;
}
static bool del_record(int i, uint8_t slot)
{
    const uint8_t del[5] = { 0x01, slot, 0x00, 0x00, 0x00 };
    return filt_op[i] == VCX_OP_ADDFILT && filt_len[i] == 5 && !memcmp(filt_rec[i], del, 5);
}
static void channel(uint32_t proto)
{
    dev_channels_clear();
    dev_channel_add(1, proto);
    memset(s_cfg, 0, sizeof(s_cfg)); memset(s_cfg_n, 0, sizeof(s_cfg_n));
    memset(s_pins_set, 0, sizeof(s_pins_set));
    memset(s_filt_n, 0, sizeof(s_filt_n));
    memset(s_block, 0, sizeof(s_block));
    memset(s_periodic, 0, sizeof(s_periodic));
    s_uart_pin[0] = 7;
    cfg_set(0, J2534_CFG_DATA_RATE, 500000);
    calls = param_len = 0; params_result = 0;
}
static long set(const uint32_t *pairs, unsigned count)
{
    uint8_t req[400] = {0};
    wr32le(req, 1); wr32le(req + 4, J2534_IOCTL_SET_CONFIG); wr32le(req + 8, count);
    for (unsigned i = 0; i < count * 2; i++) wr32le(req + 12 + i * 4, pairs[i]);
    return do_ioctl(req, (uint16_t)(12 + 8 * count), NULL, NULL, 0);
}
int main(void)
{
    dev_init(GetModuleHandle(NULL));
#ifndef VCX_RESEARCH_CONFIG
    CHECK(!dev_strict_validation());
    CHECK(s_prog_policy == PROG_V_STRICT);
    CHECK(s_license_refresh_ms == 240000);
#endif
    CHECK(proto_to_engine(VCXID_TP16) == 0x8301);
    CHECK(proto_to_engine(VCXID_KW82) == 0x9B04);
    CHECK(pt_proto_is_can(VCXID_TP16)); CHECK(pt_proto_is_kline(VCXID_KW82));
    uint8_t b[24]; int n = 0;
    emit_connect_flag_params(b, &n, sizeof(b), 0x8101, J2534_CONNECT_CAN_29BIT_ID);
    CHECK(n == 0); /* Must never write P3MIN as an ID-width flag. */
    emit_connect_flag_params(b, &n, sizeof(b), 0x9204, 0);
    CHECK(n == 0); /* KW1281 rejects checksum PID. */
    channel(J2534_ISO15765);
    const uint32_t native[] = {VCX_CFG_ISO15765_WAIT_MULT, 10, VCX_CFG_ISO15765_TIMEOUT_US, 100000};
    const uint8_t expected[] = {0x80,9,0,0,0,10, 0x80,10,0,1,0x86,0xA0};
    CHECK(set(native, 2) == 0); CHECK(param_len == sizeof(expected));
    CHECK(memcmp(params, expected, sizeof(expected)) == 0);

    /* Five-baud windows reach the firmware's PID_SLOW_* comparams in us. */
    channel(J2534_ISO14230);
    const uint32_t w1[] = {J2534_CFG_W1, 300, J2534_CFG_W4, 50};
    const uint8_t w1_wire[] = {0,0x49,0,0x04,0x93,0xE0, 0,0x4C,0,0,0xC3,0x50};
    CHECK(set(w1, 2) == 0); CHECK(param_len == sizeof(w1_wire));
    CHECK(memcmp(params, w1_wire, sizeof(w1_wire)) == 0);
    channel(J2534_2_UART_ECHO_BYTE_PS);   /* HDS's EPS timing, 09-28 live log */
    const uint32_t ueb[] = {J2534_2_CFG_UEB_T1_MAX, 1000, J2534_2_CFG_UEB_T2_MAX, 200,
                            J2534_2_CFG_UEB_T3_MAX, 120};
    const uint8_t ueb_wire[] = {0,0x49,0,0x0F,0x42,0x40, 0,0x4A,0,0x03,0x0D,0x40,
                                0,0x4B,0,0x01,0xD4,0xC0};
    CHECK(set(ueb, 3) == 0); CHECK(param_len == sizeof(ueb_wire));
    CHECK(memcmp(params, ueb_wire, sizeof(ueb_wire)) == 0);
    channel(VCXID_LIN);                   /* no slow init on LIN: nothing to push */
    CHECK(set(w1, 1) == 0); CHECK(calls == 0);
    channel(J2534_ISO9141);
    const uint32_t w1_wrap[] = {J2534_CFG_W1, 32768}, w2_wrap[] = {J2534_CFG_W2, 10001};
    CHECK(set(w1_wrap, 1) == ERR_INVALID_IOCTL_VALUE);
    CHECK(set(w2_wrap, 1) == ERR_INVALID_IOCTL_VALUE); CHECK(calls == 0);
    const uint32_t mod0[] = {J2534_CFG_FIVE_BAUD_MOD, 0}, mod1[] = {J2534_CFG_FIVE_BAUD_MOD, 1},
                   mod2[] = {J2534_CFG_FIVE_BAUD_MOD, 2}, mod4[] = {J2534_CFG_FIVE_BAUD_MOD, 4};
    CHECK(set(mod0, 1) == 0);
    CHECK(set(mod1, 1) == ERR_NOT_SUPPORTED); CHECK(set(mod2, 1) == ERR_NOT_SUPPORTED);
    CHECK(set(mod4, 1) == ERR_INVALID_IOCTL_VALUE);
    channel(J2534_ISO15765);              /* back to the native-control channel */
    CHECK(set(native, 2) == 0);
    CHECK(cfg_get(0, VCX_CFG_ISO15765_TIMEOUT_US) == 100000);
    const uint32_t native_repin[] = {J2534_2_CFG_J1962_PINS, 0x030B};
    CHECK(set(native_repin, 1) == 0);
    CHECK(param_len >= 18 && memcmp(params + 6, expected, sizeof(expected)) == 0);
    CHECK(pt_validate_config(J2534_ISO15765, VCX_CFG_ISO15765_WAIT_MULT, 256, false) == ERR_INVALID_IOCTL_VALUE);
    CHECK(pt_validate_config(J2534_ISO15765, VCX_CFG_ISO15765_TIMEOUT_US, 8421505, false) == ERR_INVALID_IOCTL_VALUE);
    CHECK(pt_validate_config(J2534_ISO15765, 0x1800B, 0, false) == ERR_NOT_SUPPORTED);
    channel(J2534_CAN);
    CHECK(set(native, 2) == ERR_NOT_SUPPORTED); CHECK(calls == 0);
    const uint32_t wrong_family[] = {J2534_CFG_P3_MIN, 10, J2534_CFG_PARITY, 2};
    CHECK(set(wrong_family, 2) == 0); CHECK(calls == 0);
    const uint32_t repin[] = {J2534_2_CFG_J1962_PINS, 0x030B};
    CHECK(set(repin, 1) == 0);
    for (int i = 0; i < param_len; i += 6) CHECK(!(params[i] == 4 && params[i+1] == 0));
    channel(J2534_ISO9141);
    const uint32_t rejected[] = {J2534_CFG_P3_MIN, 99, J2534_2_CFG_J1962_PINS, 0x0F00};
    CHECK(set(rejected, 2) == ERR_NOT_SUPPORTED);
    CHECK(cfg_get(0, J2534_CFG_P3_MIN) == 0); CHECK(calls == 0);
    const uint32_t timing[] = {J2534_CFG_P3_MIN, 25};
    CHECK(set(timing, 1) == 0);
    const uint8_t us[] = {0,0x43,0,0,0x61,0xA8};
    CHECK(param_len == 6 && memcmp(params, us, 6) == 0);
    /* HDS around a repeat DTC clear (live R001-R003): GET P1_MAX, SET 20, then
     * SET the value it read.  GET must report the J2534 default, and a 0 must
     * never reach the firmware as a 0 us inter-byte limit. */
    channel(J2534_ISO9141);
    {
        uint8_t req[16] = {0}, resp[8] = {0}; uint16_t rl = 0;
        wr32le(req, 1); wr32le(req + 4, J2534_IOCTL_GET_CONFIG); wr32le(req + 8, 1);
        wr32le(req + 12, J2534_CFG_P1_MAX);
        CHECK(do_ioctl(req, sizeof(req), resp, &rl, sizeof(resp)) == 0);
        CHECK(rl == 8 && rd32le(resp + 4) == 20);
    }
    const uint32_t p1_clear[] = {J2534_CFG_P1_MAX, 20}, p1_zero[] = {J2534_CFG_P1_MAX, 0};
    const uint8_t p1_20ms[] = {0,0x41,0,0,0x4E,0x20};
    CHECK(set(p1_clear, 1) == 0); CHECK(param_len == 6 && memcmp(params, p1_20ms, 6) == 0);
    CHECK(set(p1_zero, 1) == 0);  CHECK(param_len == 6 && memcmp(params, p1_20ms, 6) == 0);
    CHECK(cfg_get(0, J2534_CFG_P1_MAX) == 20);
    channel(J2534_ISO9141);
    params_result = 1;
    s_filt_n[0] = 1; s_block[0][0].in_use = true;
    CHECK(set(timing, 1) == ERR_FAILED); CHECK(!dev_channel_find(1));
    CHECK(s_filt_n[0] == 0 && !s_block[0][0].in_use);
    CHECK(set(timing, 1) == ERR_INVALID_CHANNEL_ID);
    channel(J2534_ISO9141); params_result = -1;
    CHECK(set(timing, 1) == -1); CHECK(!dev_channel_find(1));
    channel(J2534_CAN);
    s_filt_n[0] = 1;
    CHECK(set(repin, 1) == ERR_CHANNEL_IN_USE); CHECK(calls == 0);
    CHECK(!s_pins_set[0]); CHECK(cfg_get(0, J2534_2_CFG_J1962_PINS) == 0);
    channel(J2534_CAN); s_periodic[0].in_use = true; s_periodic[0].chan = 0;
    CHECK(set(repin, 1) == ERR_CHANNEL_IN_USE); CHECK(calls == 0);
    channel(J2534_CAN);
    for (unsigned i = 0; i < 39; i++) cfg_set(0, 0x20000 + i, i);
    const uint32_t overflow[] = {0x30000, 1};
    CHECK(set(overflow, 1) == ERR_EXCEEDED_LIMIT); CHECK(calls == 0);
    CHECK(pt_validate_config(J2534_ISO9141, J2534_CFG_P3_MIN, 4294968u, false) == ERR_INVALID_IOCTL_VALUE);
    /* Each flow-control filter owns a firmware slot.  FORScan on MS-CAN adds
     * 0x72F, 0x72E (GEM) then 0x728 (IC); when all three went out as slot 0 the
     * IC's record replaced the GEM's and the GEM's replies were dropped. */
    {
        channel(J2534_ISO15765); filt_ops = 0;
        uint32_t f727 = 0, f726 = 0, f720 = 0;
        CHECK(fc_filter(0x72F, 0x727, &f727) == 0);
        CHECK(fc_filter(0x72E, 0x726, &f726) == 0);
        CHECK(fc_filter(0x728, 0x720, &f720) == 0);
        CHECK(filt_ops == 3);
        CHECK(fc_record(0, 0, 0x72F)); CHECK(fc_record(1, 1, 0x72E)); CHECK(fc_record(2, 2, 0x728));
        CHECK(f727 && f726 && f720 && f727 != f726 && f726 != f720);
        /* The slot byte is not part of a filter's identity. */
        CHECK(fc_filter(0x72E, 0x726, NULL) == ERR_NOT_UNIQUE); CHECK(filt_ops == 3);
        /* Removing the first shifts the rest down a slot and deletes the
         * vacated last slot: FILTINIT mode 1 leaves slot contents in place. */
        uint8_t stop[8]; wr32le(stop, 1); wr32le(stop + 4, f727); filt_ops = 0;
        CHECK(do_stop_filter(stop) == 0);
        CHECK(filt_ops == 4);
        CHECK(filt_op[0] == VCX_OP_FILTINIT && filt_len[0] == 1 && filt_rec[0][0] == 0x01);
        CHECK(fc_record(1, 0, 0x72E)); CHECK(fc_record(2, 1, 0x728)); CHECK(del_record(3, 2));
        /* The next add takes the first free slot. */
        filt_ops = 0;
        CHECK(fc_filter(0x72F, 0x727, NULL) == 0); CHECK(filt_ops == 1 && fc_record(0, 2, 0x72F));
        /* CLEAR_MSG_FILTERS deletes every slot that was in use. */
        filt_ops = 0;
        CHECK(do_clear_filters(1) == 0); CHECK(s_filt_n[0] == 0);
        CHECK(filt_ops == 4 && filt_op[0] == VCX_OP_FILTINIT);
        CHECK(del_record(1, 0)); CHECK(del_record(2, 1)); CHECK(del_record(3, 2));
    }
    /* PASS records carry pattern then mask, the order the firmware reads
     * (list_msg, mask_msg).  Honda HDS: 29-bit mask FF FF FF 00, pattern 18 DA F1 00. */
    {
        channel(J2534_CAN); filt_ops = 0;
        uint8_t req[18 + 8] = {0}, resp[4]; uint16_t rl = 0;
        wr32le(req, 1); wr32le(req + 4, J2534_PASS_FILTER); wr32le(req + 8, 0x100);
        req[12] = 4; req[14] = 4;
        const uint8_t mask[4] = {0xFF, 0xFF, 0xFF, 0x00}, patt[4] = {0x18, 0xDA, 0xF1, 0x00};
        memcpy(req + 18, mask, 4); memcpy(req + 22, patt, 4);
        CHECK(do_filter(req, sizeof(req), resp, &rl, sizeof(resp)) == 0);
        const uint8_t want[13] = {0x01, 0x00, 0x01, 0x01, 0x04,
                                  0x18, 0xDA, 0xF1, 0x00, 0xFF, 0xFF, 0xFF, 0x00};
        CHECK(filt_ops == 1 && filt_op[0] == VCX_OP_ADDFILT && filt_len[0] == 13 &&
              !memcmp(filt_rec[0], want, 13));
        dev_channels_clear();
    }
    /* Configuration cannot change underneath an active host. */
    CHECK(ini_valid("port", "COM5"));
    CHECK(!ini_valid("port", "COM5junk"));
    CHECK(!ini_valid("hex_max", "-1"));
    CHECK(!ini_valid("rx_log_every", "0"));
    CHECK(!ini_valid("log_level", "3"));
    CHECK(!ini_valid("license_refresh_ms", "240000junk"));
    CHECK(!ini_valid("can_bus", "garbage"));
    CHECK(!ini_valid("kline_pin", "257"));
    _putenv("VCX_NANO_PORT=COM17");
    _putenv("VCX_NANO_STRICT=1");
    _putenv("VCX_NANO_VOLTAGE_POLICY=compat");
    _putenv("VCX_NANO_LICENSE_REFRESH_MS=0");
    ini_load(NULL);
    char option[MAX_PATH];
    CHECK(ini_get("port", option, sizeof(option)) && !strcmp(option, "COM17"));
    _putenv("VCX_NANO_PORT=COM18");
    CHECK(ini_get("port", option, sizeof(option)) && !strcmp(option, "COM17"));
#ifdef VCX_RESEARCH_CONFIG
    CHECK(ini_get("strict_validation", option, sizeof(option)) && !strcmp(option, "1"));
    CHECK(ini_get("voltage_policy", option, sizeof(option)) && !strcmp(option, "compat"));
    CHECK(ini_get("license_refresh_ms", option, sizeof(option)) && !strcmp(option, "0"));
#else
    CHECK(!ini_get("strict_validation", option, sizeof(option)));
    CHECK(!ini_get("voltage_policy", option, sizeof(option)));
    CHECK(!ini_get("license_refresh_ms", option, sizeof(option)));
    CHECK(!ini_get("can_bus", option, sizeof(option)));
    CHECK(!ini_get("kline_pin", option, sizeof(option)));
    CHECK(!ini_get("periodic", option, sizeof(option)));
#endif
    _putenv("VCX_NANO_PORT=COM18junk");
    ini_load(NULL);
    CHECK(!ini_get("port", option, sizeof(option)));
    CHECK(s_ini[0].invalid);
    _putenv("VCX_NANO_PORT=");
    _putenv("VCX_NANO_STRICT=");
    _putenv("VCX_NANO_VOLTAGE_POLICY=");
    _putenv("VCX_NANO_LICENSE_REFRESH_MS=");
    dev_channels_clear();
    printf("config wire tests: %s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
