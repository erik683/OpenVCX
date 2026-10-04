/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * pt_validate.h - centralized J2534-1 (04.04) semantic validation.
 *
 * The DLL's two layers (api.c shape checks, device_vcx.c wire framing) each grew
 * their own ad-hoc bounds, so the spec's error contract fell through the crack:
 * ERR_INVALID_FLAGS, ERR_INVALID_BAUDRATE, ERR_INVALID_IOCTL_ID, ERR_NOT_UNIQUE
 * and ERR_NO_FLOW_CONTROL were defined but never returned.  This module is the
 * single home for those rules.  It is pure C over j2534_defs.h -- no Windows, no
 * device state -- so validate_test.c can exercise every rule on the host with no
 * hardware.
 *
 * Vendor-compatibility policy: the VCX Nano is used with real tools (FORScan,
 * HDS) whose flows the vendor VCXPT32.dll tolerated even where they bend the
 * spec.  So each rule is either HARD (a NULL, a value that would overflow wire
 * framing, or one the vendor also rejects -- always enforced) or STRICT-ONLY
 * (spec-illegal but vendor-tolerated -- enforced only when `strict`, otherwise
 * the caller logs it and proceeds).  `strict` comes from the strict_validation
 * ini flag (default 0 = lenient), threaded in by dev_strict_validation().
 *
 * Every function returns a J2534 SAE code: STATUS_NOERROR (0) when the input is
 * acceptable, otherwise the specific ERR_* the spec assigns to that failure.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "j2534_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The J2534 operation a PASSTHRU_MSG is being validated for.  The per-protocol
 * length bounds differ by operation: a periodic message is capped at a single
 * bus frame while an ISO15765 write may segment up to 4128 bytes. */
typedef enum {
    PT_OP_WRITE,
    PT_OP_PERIODIC,
    PT_OP_FAST_INIT
} pt_op_t;

/* Connect Flags legal for the protocol (e.g. CAN_29BIT_ID only on CAN-family,
 * ISO9141_NO_CHECKSUM only on K-line).  Illegal bit for the protocol, or an
 * undefined bit -> ERR_INVALID_FLAGS.  The device-private top-byte flags
 * (PT_CONNECT_LISTEN_ONLY/SELF_TEST) are always allowed. */
long pt_validate_connect_flags(uint32_t proto, uint32_t flags, bool strict);

/* BaudRate sanity.  Hard: reject 0 and physically-impossible rates.  Strict:
 * require one of the standard rates for the protocol family. -> ERR_INVALID_BAUDRATE. */
long pt_validate_baudrate(uint32_t proto, uint32_t baud, bool strict);

/* PASSTHRU_MSG length bounds by protocol and operation.  `data_size` is the
 * caller's full unsigned value taken BEFORE any 16-bit cast, so an oversize
 * DataSize is caught instead of wrapping (the api.c:58 truncation hazard).
 * -> ERR_INVALID_MSG. */
long pt_validate_msg(uint32_t proto, pt_op_t op, uint32_t data_size,
                     uint32_t txflags, bool strict);

/* SET_CONFIG parameter/value legality.  Structurally-invalid value for a known
 * param -> ERR_INVALID_IOCTL_VALUE (hard).  Unknown param, or a param that does
 * not apply to the protocol -> ERR_NOT_SUPPORTED (strict only; default returns
 * NOERROR so the device caches it cache-only, matching the vendor). */
long pt_validate_config(uint32_t proto, uint32_t param, uint32_t value,
                        bool strict);

/* Filter type/length legality.  Bad type -> ERR_NOT_SUPPORTED; FLOW_CONTROL on a
 * non-CAN protocol -> ERR_NOT_SUPPORTED; out-of-range mask/pattern/fc widths ->
 * ERR_INVALID_MSG.  Lengths are full unsigned values (truncation-safe). */
long pt_validate_filter(uint32_t proto, uint32_t ftype, uint32_t mlen,
                        uint32_t plen, uint32_t fclen, bool strict);

/* Protocol-family classifiers, exported so the callers and the unit test share
 * one definition of "is this CAN?" instead of re-listing the _PS aliases. */
bool pt_proto_is_can(uint32_t proto);
bool pt_proto_is_iso15765(uint32_t proto);
bool pt_proto_is_kline(uint32_t proto);
bool pt_proto_is_j1850(uint32_t proto);

#ifdef __cplusplus
}
#endif
