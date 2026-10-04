/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * pt_validate.c - see pt_validate.h.  Pure C over j2534_defs.h; no device state.
 */
#include "pt_validate.h"

/* ---- protocol family classifiers ---- */

bool pt_proto_is_can(uint32_t p)
{
    switch (p) {
    case J2534_CAN:
    case J2534_ISO15765:
    case J2534_2_CAN_PS:
    case J2534_2_ISO15765_PS:
    case J2534_2_SW_CAN_PS:
    case J2534_2_SW_ISO15765_PS:
    case J2534_2_FT_CAN_PS:
    case J2534_2_FT_ISO15765_PS:
    case J2534_2_TP2_0_PS:
    /* Device-private ids that proto_to_engine() maps onto CAN engines: TP2.0
     * (0x8021 -> 0x8401) and J1939_PS (0x800C -> 0x8201).  They belong here
     * because do_filter's own engine_is_can() gate moved into this function --
     * omitting them would reject a flow-control filter the device accepts. */
    case VCXID_TP16:
    case 0x8021u:
    case 0x800Cu:
        return true;
    default:
        return false;
    }
}

bool pt_proto_is_iso15765(uint32_t p)
{
    switch (p) {
    case J2534_ISO15765:
    case J2534_2_ISO15765_PS:
    case J2534_2_SW_ISO15765_PS:
    case J2534_2_FT_ISO15765_PS:
        return true;
    default:
        return false;
    }
}

bool pt_proto_is_kline(uint32_t p)
{
    switch (p) {
    case J2534_ISO9141:
    case J2534_ISO14230:
    case J2534_2_ISO9141_PS:
    case J2534_2_ISO14230_PS:
    case J2534_2_HONDA_DIAGH_PS:
    case J2534_2_UART_ECHO_BYTE_PS:
    /* Device-private LIN (0x8020).  proto_to_engine() maps it onto UART engine
     * 0x9D04 and device_vcx.c's engine_is_kline() matches that engine, so
     * leaving LIN out here made the validator and the device layer disagree:
     * strict mode refused DATA_BITS/PARITY and the K-line connect flags on a
     * channel whose engine would have consumed them. */
    case VCXID_KW82:
    case 0x8020u:
        return true;
    default:
        return false;
    }
}

/* Protocols whose engine runs a segmenting transport layer, so a single J2534
 * write may legitimately be far longer than one bus frame.  ISO15765 is the
 * obvious one; TP2.0 has its own TP layer and J1939 segments via BAM/TP up to
 * 1785 bytes.  All three are inside pt_proto_is_can(), so without this the raw
 * CAN one-frame ceiling would be applied to them. */
static bool proto_segments(uint32_t p)
{
    if (pt_proto_is_iso15765(p)) return true;
    switch (p) {
    case J2534_2_TP2_0_PS:
    case VCXID_TP16:
    case 0x8021u:   /* VCXID_TP20 */
    case 0x800Cu:   /* VCXID_J1939_PS */
        return true;
    default:
        return false;
    }
}

bool pt_proto_is_j1850(uint32_t p)
{
    switch (p) {
    case J2534_J1850VPW:
    case J2534_J1850PWM:
    case J2534_2_J1850VPW_PS:
    case J2534_2_J1850PWM_PS:
        return true;
    default:
        return false;
    }
}

/* A protocol this validator recognizes at all.  The device-private ids live in
 * device_vcx.c; only the ones it maps onto a CAN engine are listed above,
 * because the flow-control rule is hard.  The rest (LIN 0x8020, J1708_PS
 * 0x800D, ...) are not listed here; for those, family classifiers all return
 * false and the strict-only
 * rules default to permissive, leaving proto_to_engine() in do_connect as the
 * authority on whether the protocol exists (ERR_INVALID_PROTOCOL_ID). */
static bool proto_known(uint32_t p)
{
    return pt_proto_is_can(p) || pt_proto_is_kline(p) || pt_proto_is_j1850(p);
}

/* ---- connect flags ---- */

long pt_validate_connect_flags(uint32_t proto, uint32_t flags, bool strict)
{
    /* Device-private top-byte flags are always allowed and carry no protocol
     * restriction; strip them before checking the J2534 flag bits. */
    uint32_t f = flags & ~(PT_CONNECT_LISTEN_ONLY | PT_CONNECT_SELF_TEST);

    const uint32_t can_flags = J2534_CONNECT_CAN_29BIT_ID |
                               J2534_CONNECT_CAN_ID_BOTH;
    const uint32_t kline_flags = J2534_CONNECT_ISO9141_NO_CHECKSUM |
                                 J2534_CONNECT_ISO9141_K_LINE_ONLY;
    const uint32_t all_known = can_flags | kline_flags;

    /* Undefined flag bits: hard reject only what cannot be a real flag.  The
     * vendor tolerated stray bits, so gate the generic "unknown bit" check on
     * strict; but a bit outside the whole known set with strict off is still
     * just forwarded. */
    if (strict && (f & ~all_known))
        return ERR_INVALID_FLAGS;

    /* CAN addressing flags on a non-CAN protocol, or K-line checksum/line flags
     * on a non-K-line protocol, are meaningless -- the classic ERR_INVALID_FLAGS
     * case.  Strict-only: the vendor forwarded these and let the engine ignore
     * them. */
    if (strict) {
        if ((f & can_flags) && !pt_proto_is_can(proto))
            return ERR_INVALID_FLAGS;
        if ((f & kline_flags) && !pt_proto_is_kline(proto))
            return ERR_INVALID_FLAGS;
    }
    return STATUS_NOERROR;
}

/* ---- baud rate ---- */

static bool baud_in_set(uint32_t b, const uint32_t *set, int n)
{
    for (int i = 0; i < n; i++) if (set[i] == b) return true;
    return false;
}

long pt_validate_baudrate(uint32_t proto, uint32_t baud, bool strict)
{
    /* Hard: a zero or absurd rate is not a rate on any bus this device drives.
     * 4 Mbit/s is comfortably above CAN-FD-less high-speed CAN (1 Mbit/s). */
    if (baud == 0 || baud > 4000000u)
        return ERR_INVALID_BAUDRATE;
    if (!strict || !proto_known(proto))
        return STATUS_NOERROR;

    /* Strict: the standard rate set for the family.  Single-wire CAN adds its
     * 33.3 kbit/s and 83.3 kbit/s high-speed mode; K-line and J1850 have fixed
     * defaults but tools do vary the K-line rate, so allow the common set. */
    if (pt_proto_is_can(proto)) {
        static const uint32_t s[] = {33333u, 50000u, 83333u, 100000u, 125000u,
                                     250000u, 500000u, 1000000u};
        return baud_in_set(baud, s, 8) ? STATUS_NOERROR : ERR_INVALID_BAUDRATE;
    }
    if (pt_proto_is_kline(proto)) {
        static const uint32_t s[] = {9600u, 10400u, 15625u, 19200u, 38400u,
                                     57600u, 115200u};
        return baud_in_set(baud, s, 7) ? STATUS_NOERROR : ERR_INVALID_BAUDRATE;
    }
    if (pt_proto_is_j1850(proto)) {
        /* VPW 10.4 kbit/s, PWM 41.6 kbit/s. */
        static const uint32_t s[] = {10400u, 10416u, 41600u};
        return baud_in_set(baud, s, 3) ? STATUS_NOERROR : ERR_INVALID_BAUDRATE;
    }
    return STATUS_NOERROR;
}

/* ---- message length by protocol/operation ---- */

long pt_validate_msg(uint32_t proto, pt_op_t op, uint32_t data_size,
                     uint32_t txflags, bool strict)
{
    (void)txflags;

    /* Periodic messages are a single bus frame regardless of protocol; the
     * firmware timer record and the host scheduler both cap payload at 12.
     * Hard on every path. */
    if (op == PT_OP_PERIODIC)
        return (data_size >= 1 && data_size <= 12) ? STATUS_NOERROR
                                                   : ERR_INVALID_MSG;

    /* FAST_INIT carries an optional StartMessage; only the buffer ceiling is
     * structural. */
    if (op == PT_OP_FAST_INIT)
        return (data_size <= J2534_MSG_DATA_MAX) ? STATUS_NOERROR : ERR_INVALID_MSG;

    /* PT_OP_WRITE.  Hard ceiling is the PASSTHRU_MSG.Data buffer; a zero-length
     * write is meaningless on every protocol. */
    if (data_size == 0 || data_size > J2534_MSG_DATA_MAX)
        return ERR_INVALID_MSG;

    if (!strict)
        return STATUS_NOERROR;

    /* Strict per-protocol bounds.  CAN-family writes carry a 4-byte CAN id
     * prefix, so the minimum is 4 (an id with no data).  The one-frame ceiling
     * (id plus eight data bytes) applies only where the engine cannot segment --
     * ISO15765, TP2.0 and J1939 all can, and capping them at 12 rejected
     * perfectly ordinary segmented requests the firmware handles. */
    if (pt_proto_is_can(proto)) {
        if (data_size < 4)
            return ERR_INVALID_MSG;
        if (!proto_segments(proto) && data_size > 12)
            return ERR_INVALID_MSG;
    }
    return STATUS_NOERROR;
}

/* ---- SET_CONFIG parameter/value ---- */

/* Is `param` a defined J2534-1/-2 config id at all? */
static bool config_param_known(uint32_t p)
{
    if (p >= J2534_CFG_DATA_RATE && p <= J2534_CFG_ISO15765_WFT_MAX)
        return true;   /* 0x01..0x25 is the contiguous J2534-1 block */
    return p == J2534_2_CFG_CAN_MIXED_FORMAT || p == J2534_2_CFG_J1962_PINS ||
           (p >= J2534_2_CFG_UEB_T0_MIN && p <= J2534_2_CFG_UEB_T3_MAX);
}

/* Params that only make sense on an ISO15765 channel. */
static bool config_param_iso15765_only(uint32_t p)
{
    switch (p) {
    case J2534_CFG_ISO15765_BS:
    case J2534_CFG_ISO15765_STMIN:
    case J2534_CFG_BS_TX:
    case J2534_CFG_STMIN_TX:
    case J2534_CFG_ISO15765_WFT_MAX:
        return true;
    default:
        return false;
    }
}

long pt_validate_config(uint32_t proto, uint32_t param, uint32_t value,
                        bool strict)
{
    switch (param) {
    case VCX_CFG_ISO15765_EXT:
    case VCX_CFG_ISO15765_TXBS:
    case VCX_CFG_ISO15765_TXSTMIN:
    case VCX_CFG_ISO15765_WAIT_MULT:
    case VCX_CFG_ISO15765_TIMEOUT_US:
        if (!pt_proto_is_iso15765(proto)) return ERR_NOT_SUPPORTED;
        if (param == VCX_CFG_ISO15765_EXT && value > 3) return ERR_INVALID_IOCTL_VALUE;
        if (param == VCX_CFG_ISO15765_TXBS && value > 255) return ERR_INVALID_IOCTL_VALUE;
        /* Bound each factor so the firmware's signed timeout product fits. */
        if (param == VCX_CFG_ISO15765_WAIT_MULT && value > 255) return ERR_INVALID_IOCTL_VALUE;
        if (param == VCX_CFG_ISO15765_TIMEOUT_US && value > 8421504u) return ERR_INVALID_IOCTL_VALUE;
        return STATUS_NOERROR;
    case J2534_CFG_P1_MAX: case J2534_CFG_P2_MAX:
    case J2534_CFG_P3_MIN: case J2534_CFG_P4_MIN:
    case J2534_CFG_TIDLE: case J2534_CFG_TWUP: case J2534_CFG_TINIL:
        if (value > 4294967u) return ERR_INVALID_IOCTL_VALUE;
        break;
    default: break;
    }

    if ((param & 0xFFFF0000u) == 0x10000u) return ERR_NOT_SUPPORTED;

    /* Structural value bounds -- enforced hard because the vendor rejects these
     * too and a bad value would be pushed straight at the firmware. */
    switch (param) {
    case J2534_CFG_LOOPBACK:
        if (value > 1) return ERR_INVALID_IOCTL_VALUE;
        break;
    case J2534_CFG_DATA_BITS:
        /* J2534-1 04.04 encodes this as 0 = 8 data bits, 1 = 7 data bits.  The
         * literal counts 7 and 8 are accepted too: uart_format_value() has
         * always consumed this field as a bit count, and tools do send it that
         * way.  Both encodings are translated in uart_format_value(). */
        if (value != 0 && value != 1 && value != 7 && value != 8)
            return ERR_INVALID_IOCTL_VALUE;
        break;
    case J2534_CFG_PARITY:
        if (value > 2) return ERR_INVALID_IOCTL_VALUE;   /* 0 none,1 odd,2 even */
        break;
    case J2534_CFG_ISO15765_BS:
    case J2534_CFG_ISO15765_STMIN:
        if (value > 0xFFu) return ERR_INVALID_IOCTL_VALUE;   /* one wire byte */
        break;
    case J2534_CFG_BS_TX:
    case J2534_CFG_STMIN_TX:
        /* One wire byte, plus the J2534-1 sentinel 0xFFFF: "use the value from
         * the received flow control frame", which is also these two params'
         * default.  Rejecting it would leave an app unable to hand pacing back
         * to the ECU once it had overridden it. */
        if (value > 0xFFu && value != 0xFFFFu) return ERR_INVALID_IOCTL_VALUE;
        break;
    /* Five-baud windows go to the firmware in microseconds and run on its
     * 2 kHz timer, whose period register is 2*ms-1 in 16 bits: W1 alone, and
     * W2+W3+W4 plus the firmware's W4max and 10 ms as one period
     * (notes/uart_slow_init_firmware.md). Larger values would wrap there. */
    case J2534_CFG_W1: case J2534_2_CFG_UEB_T1_MAX:
        if (value > 32767u) return ERR_INVALID_IOCTL_VALUE;
        break;
    case J2534_CFG_W2: case J2534_CFG_W3: case J2534_CFG_W4:
    case J2534_2_CFG_UEB_T2_MAX: case J2534_2_CFG_UEB_T3_MAX:
        if (value > 10000u) return ERR_INVALID_IOCTL_VALUE;
        break;
    case J2534_CFG_W5: case J2534_2_CFG_UEB_T0_MIN:
        if (value > 4294967u) return ERR_INVALID_IOCTL_VALUE;
        break;
    case J2534_CFG_FIVE_BAUD_MOD:
        /* The firmware picks the handshake from the key bytes and always sends
         * ~KB2, so only mode 0 (ISO 9141-2 / 14230-4) is honest to accept. */
        if (value > 3) return ERR_INVALID_IOCTL_VALUE;
        if (value != 0) return ERR_NOT_SUPPORTED;
        break;
    default:
        break;
    }

    if (!strict)
        return STATUS_NOERROR;

    /* Strict: unknown param, or a known param used on the wrong protocol, is
     * ERR_NOT_SUPPORTED.  Default mode returned NOERROR above so the device
     * caches it cache-only, exactly as the vendor DLL did. */
    if (!config_param_known(param))
        return ERR_NOT_SUPPORTED;
    if (config_param_iso15765_only(param) && !pt_proto_is_iso15765(proto))
        return ERR_NOT_SUPPORTED;
    if ((param == J2534_CFG_DATA_BITS || param == J2534_CFG_PARITY) &&
        !pt_proto_is_kline(proto))
        return ERR_NOT_SUPPORTED;
    return STATUS_NOERROR;
}

/* ---- filters ---- */

long pt_validate_filter(uint32_t proto, uint32_t ftype, uint32_t mlen,
                        uint32_t plen, uint32_t fclen, bool strict)
{
    (void)strict;

    if (ftype != J2534_PASS_FILTER && ftype != J2534_BLOCK_FILTER &&
        ftype != J2534_FLOW_CONTROL_FILTER)
        return ERR_NOT_SUPPORTED;

    /* Flow control is an ISO15765/CAN concept; the K-line and J1850 engines have
     * no TP layer to hang it on.  Hard, matching the vendor and the firmware. */
    if (ftype == J2534_FLOW_CONTROL_FILTER && !pt_proto_is_can(proto))
        return ERR_NOT_SUPPORTED;

    if (ftype == J2534_PASS_FILTER || ftype == J2534_BLOCK_FILTER) {
        /* One mask and one pattern of equal, in-range width. */
        if (mlen < 1 || mlen > 12 || plen != mlen)
            return ERR_INVALID_MSG;
    } else {   /* FLOW_CONTROL */
        /* Mask, pattern and flow-control message each at least the 4-byte CAN
         * id, and no wider than a full frame. */
        if (mlen < 4 || plen < 4 || fclen < 4 ||
            mlen > 12 || plen > 12 || fclen > 12)
            return ERR_INVALID_MSG;
    }
    return STATUS_NOERROR;
}
