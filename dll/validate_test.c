/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * validate_test.c - host-only unit tests for pt_validate.c.  No device, no DLL
 * load: this is the primary gate for the J2534 error contract, and the proof
 * that ERR_INVALID_FLAGS / ERR_INVALID_BAUDRATE / ERR_INVALID_IOCTL_ID /
 * ERR_NOT_UNIQUE / ERR_NO_FLOW_CONTROL are real codepaths and not dead #defines.
 *
 * (ERR_NOT_UNIQUE and ERR_NO_FLOW_CONTROL are enforced in device_vcx.c against
 * live channel/filter state, so they are covered by the on-hardware smoke rather
 * than here; this file proves the pure per-call rules.)
 *
 * Build: cl validate_test.c pt_validate.c ; run: validate_test.exe
 */
#include <stdio.h>

#include "pt_validate.h"

static int failures;

#define EXPECT(expr, want, desc)                                              \
    do {                                                                      \
        long got_ = (long)(expr);                                             \
        if (got_ == (long)(want)) {                                           \
            printf("  ok  : %s\n", desc);                                     \
        } else {                                                             \
            printf("  FAIL: %s (want 0x%02lX got 0x%02lX)\n", desc,           \
                   (long)(want), got_);                                       \
            failures++;                                                       \
        }                                                                     \
    } while (0)

static void test_connect_flags(void)
{
    printf("connect flags:\n");
    /* Legal combinations pass in both modes. */
    EXPECT(pt_validate_connect_flags(J2534_CAN, J2534_CONNECT_CAN_29BIT_ID, false),
           STATUS_NOERROR, "CAN + 29BIT lenient");
    EXPECT(pt_validate_connect_flags(J2534_CAN, J2534_CONNECT_CAN_29BIT_ID, true),
           STATUS_NOERROR, "CAN + 29BIT strict");
    EXPECT(pt_validate_connect_flags(J2534_ISO9141,
           J2534_CONNECT_ISO9141_NO_CHECKSUM, true),
           STATUS_NOERROR, "ISO9141 + NO_CHECKSUM strict");
    /* Device-private flags always allowed. */
    EXPECT(pt_validate_connect_flags(J2534_CAN, PT_CONNECT_LISTEN_ONLY, true),
           STATUS_NOERROR, "CAN + LISTEN_ONLY strict");

    /* Wrong-family flag: lenient tolerates, strict rejects. */
    EXPECT(pt_validate_connect_flags(J2534_CAN,
           J2534_CONNECT_ISO9141_NO_CHECKSUM, false),
           STATUS_NOERROR, "CAN + K-line checksum flag lenient (tolerated)");
    EXPECT(pt_validate_connect_flags(J2534_CAN,
           J2534_CONNECT_ISO9141_NO_CHECKSUM, true),
           ERR_INVALID_FLAGS, "CAN + K-line checksum flag strict -> INVALID_FLAGS");
    EXPECT(pt_validate_connect_flags(J2534_ISO9141,
           J2534_CONNECT_CAN_29BIT_ID, true),
           ERR_INVALID_FLAGS, "ISO9141 + CAN 29BIT strict -> INVALID_FLAGS");
    /* Undefined bit: strict only. */
    EXPECT(pt_validate_connect_flags(J2534_CAN, 0x00002000u, false),
           STATUS_NOERROR, "CAN + undefined bit lenient (tolerated)");
    EXPECT(pt_validate_connect_flags(J2534_CAN, 0x00002000u, true),
           ERR_INVALID_FLAGS, "CAN + undefined bit strict -> INVALID_FLAGS");
}

static void test_baudrate(void)
{
    printf("baud rate:\n");
    EXPECT(pt_validate_baudrate(J2534_CAN, 500000u, false), STATUS_NOERROR,
           "CAN 500k lenient");
    EXPECT(pt_validate_baudrate(J2534_CAN, 500000u, true), STATUS_NOERROR,
           "CAN 500k strict");
    /* Zero and absurd are hard rejects. */
    EXPECT(pt_validate_baudrate(J2534_CAN, 0u, false), ERR_INVALID_BAUDRATE,
           "CAN 0 lenient -> INVALID_BAUDRATE");
    EXPECT(pt_validate_baudrate(J2534_CAN, 9000000u, false), ERR_INVALID_BAUDRATE,
           "CAN 9M lenient -> INVALID_BAUDRATE");
    /* Non-standard rate: tolerated lenient, rejected strict. */
    EXPECT(pt_validate_baudrate(J2534_CAN, 456700u, false), STATUS_NOERROR,
           "CAN 456700 lenient (tolerated)");
    EXPECT(pt_validate_baudrate(J2534_CAN, 456700u, true), ERR_INVALID_BAUDRATE,
           "CAN 456700 strict -> INVALID_BAUDRATE");
    EXPECT(pt_validate_baudrate(J2534_ISO9141, 10400u, true), STATUS_NOERROR,
           "ISO9141 10400 strict");
}

static void test_msg(void)
{
    printf("message length:\n");
    /* Write: hard buffer ceiling, zero rejected. */
    EXPECT(pt_validate_msg(J2534_CAN, PT_OP_WRITE, 0u, 0u, false),
           ERR_INVALID_MSG, "CAN write 0 -> INVALID_MSG");
    EXPECT(pt_validate_msg(J2534_ISO15765, PT_OP_WRITE, 4128u, 0u, false),
           STATUS_NOERROR, "ISO15765 write 4128 lenient");
    EXPECT(pt_validate_msg(J2534_ISO15765, PT_OP_WRITE, 4129u, 0u, false),
           ERR_INVALID_MSG, "ISO15765 write 4129 -> INVALID_MSG (buffer)");
    /* Truncation hazard: a value that wraps to a small u16 must still fail. */
    EXPECT(pt_validate_msg(J2534_CAN, PT_OP_WRITE, 0x10004u, 0u, false),
           ERR_INVALID_MSG, "CAN write 0x10004 (wraps to 4) -> INVALID_MSG");
    /* Strict per-protocol: raw CAN capped at 12, ISO15765 not. */
    EXPECT(pt_validate_msg(J2534_CAN, PT_OP_WRITE, 20u, 0u, false),
           STATUS_NOERROR, "raw CAN write 20 lenient (tolerated)");
    EXPECT(pt_validate_msg(J2534_CAN, PT_OP_WRITE, 20u, 0u, true),
           ERR_INVALID_MSG, "raw CAN write 20 strict -> INVALID_MSG");
    EXPECT(pt_validate_msg(J2534_CAN, PT_OP_WRITE, 3u, 0u, true),
           ERR_INVALID_MSG, "raw CAN write 3 strict (< id) -> INVALID_MSG");
    EXPECT(pt_validate_msg(J2534_ISO15765, PT_OP_WRITE, 20u, 0u, true),
           STATUS_NOERROR, "ISO15765 write 20 strict (segments)");
    /* TP2.0 and J1939 are inside pt_proto_is_can() but run their own transport
     * layers, so the one-frame ceiling must not apply to them either. */
    EXPECT(pt_validate_msg(J2534_2_TP2_0_PS, PT_OP_WRITE, 40u, 0u, true),
           STATUS_NOERROR, "TP2.0 write 40 strict (segments)");
    EXPECT(pt_validate_msg(0x800Cu, PT_OP_WRITE, 40u, 0u, true),
           STATUS_NOERROR, "J1939_PS write 40 strict (segments)");
    /* Periodic: 1..12 hard on every protocol. */
    EXPECT(pt_validate_msg(J2534_CAN, PT_OP_PERIODIC, 12u, 0u, false),
           STATUS_NOERROR, "CAN periodic 12");
    EXPECT(pt_validate_msg(J2534_CAN, PT_OP_PERIODIC, 13u, 0u, false),
           ERR_INVALID_MSG, "CAN periodic 13 -> INVALID_MSG");
    /* Fast init: buffer ceiling only, empty allowed. */
    EXPECT(pt_validate_msg(J2534_ISO14230, PT_OP_FAST_INIT, 0u, 0u, false),
           STATUS_NOERROR, "ISO14230 fast-init empty ok");
    EXPECT(pt_validate_msg(J2534_ISO14230, PT_OP_FAST_INIT, 5000u, 0u, false),
           ERR_INVALID_MSG, "ISO14230 fast-init 5000 -> INVALID_MSG");
}

static void test_config(void)
{
    printf("config:\n");
    /* Structural value bounds: hard in both modes. */
    EXPECT(pt_validate_config(J2534_CAN, J2534_CFG_LOOPBACK, 1u, false),
           STATUS_NOERROR, "LOOPBACK 1 ok");
    EXPECT(pt_validate_config(J2534_CAN, J2534_CFG_LOOPBACK, 2u, false),
           ERR_INVALID_IOCTL_VALUE, "LOOPBACK 2 -> INVALID_IOCTL_VALUE");
    EXPECT(pt_validate_config(J2534_ISO9141, J2534_CFG_DATA_BITS, 8u, false),
           STATUS_NOERROR, "DATA_BITS 8 ok");
    EXPECT(pt_validate_config(J2534_ISO9141, J2534_CFG_DATA_BITS, 9u, false),
           ERR_INVALID_IOCTL_VALUE, "DATA_BITS 9 -> INVALID_IOCTL_VALUE");
    EXPECT(pt_validate_config(J2534_ISO15765, J2534_CFG_ISO15765_STMIN, 300u, false),
           ERR_INVALID_IOCTL_VALUE, "STMIN 300 -> INVALID_IOCTL_VALUE (>1 byte)");
    /* Unknown param: cache-only lenient, NOT_SUPPORTED strict. */
    EXPECT(pt_validate_config(J2534_CAN, 0x00007777u, 1u, false),
           STATUS_NOERROR, "unknown param lenient (cache-only)");
    EXPECT(pt_validate_config(J2534_CAN, 0x00007777u, 1u, true),
           ERR_NOT_SUPPORTED, "unknown param strict -> NOT_SUPPORTED");
    /* Wrong-protocol param: strict only. */
    EXPECT(pt_validate_config(J2534_CAN, J2534_CFG_ISO15765_BS, 8u, false),
           STATUS_NOERROR, "ISO15765_BS on CAN lenient (tolerated)");
    EXPECT(pt_validate_config(J2534_CAN, J2534_CFG_ISO15765_BS, 8u, true),
           ERR_NOT_SUPPORTED, "ISO15765_BS on CAN strict -> NOT_SUPPORTED");
    EXPECT(pt_validate_config(J2534_ISO15765, J2534_CFG_ISO15765_BS, 8u, true),
           STATUS_NOERROR, "ISO15765_BS on ISO15765 strict ok");
    EXPECT(pt_validate_config(J2534_CAN, J2534_CFG_DATA_BITS, 8u, true),
           ERR_NOT_SUPPORTED, "DATA_BITS on CAN strict -> NOT_SUPPORTED");
    /* LIN (device-private 0x8020) runs on UART engine 0x9D04, which
     * device_vcx.c's engine_is_kline() matches -- the validator must agree. */
    EXPECT(pt_validate_config(0x8020u, J2534_CFG_DATA_BITS, 8u, true),
           STATUS_NOERROR, "DATA_BITS on LIN strict ok (LIN is K-line)");
    EXPECT(pt_validate_connect_flags(0x8020u,
           J2534_CONNECT_ISO9141_NO_CHECKSUM, true),
           STATUS_NOERROR, "LIN + NO_CHECKSUM strict ok");
}

static void test_filter(void)
{
    printf("filter:\n");
    EXPECT(pt_validate_filter(J2534_CAN, J2534_PASS_FILTER, 4u, 4u, 0u, false),
           STATUS_NOERROR, "CAN pass 4/4 ok");
    EXPECT(pt_validate_filter(J2534_CAN, J2534_PASS_FILTER, 4u, 5u, 0u, false),
           ERR_INVALID_MSG, "CAN pass 4/5 (mismatch) -> INVALID_MSG");
    EXPECT(pt_validate_filter(J2534_CAN, J2534_PASS_FILTER, 0u, 0u, 0u, false),
           ERR_INVALID_MSG, "CAN pass 0/0 -> INVALID_MSG");
    EXPECT(pt_validate_filter(J2534_CAN, J2534_BLOCK_FILTER, 4u, 4u, 0u, false),
           STATUS_NOERROR, "CAN block 4/4 ok (now supported)");
    EXPECT(pt_validate_filter(J2534_ISO15765, J2534_FLOW_CONTROL_FILTER,
           4u, 4u, 4u, false), STATUS_NOERROR, "ISO15765 flow-control 4/4/4 ok");
    EXPECT(pt_validate_filter(J2534_ISO15765, J2534_FLOW_CONTROL_FILTER,
           4u, 4u, 3u, false), ERR_INVALID_MSG, "flow-control fc=3 -> INVALID_MSG");
    /* Flow control on a K-line protocol is unsupported. */
    EXPECT(pt_validate_filter(J2534_ISO9141, J2534_FLOW_CONTROL_FILTER,
           4u, 4u, 4u, false), ERR_NOT_SUPPORTED,
           "flow-control on ISO9141 -> NOT_SUPPORTED");
    /* Bad filter type. */
    EXPECT(pt_validate_filter(J2534_CAN, 99u, 4u, 4u, 0u, false),
           ERR_NOT_SUPPORTED, "filter type 99 -> NOT_SUPPORTED");
    /* Truncation-safe width. */
    EXPECT(pt_validate_filter(J2534_CAN, J2534_PASS_FILTER, 0x10004u, 0x10004u,
           0u, false), ERR_INVALID_MSG, "pass width 0x10004 -> INVALID_MSG");
}

int main(void)
{
    test_connect_flags();
    test_baudrate();
    test_msg();
    test_config();
    test_filter();
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
           failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
