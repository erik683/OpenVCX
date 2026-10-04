/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * vcx_transport.h - VCX Nano wire framing: frame builder and receive parser.
 *
 *   frame  : BB <escaped(content)> BB
 *   content: 80 <cmd_hi> <cmd_lo> <chan> [payload...] <check>
 *   check  : sum(all content bytes) & 0xFF
 *   escape : any content byte in {BB,EE,DD} -> DD (~byte)
 *
 * Pure byte handling -- no port, clock, locks or logging -- so the reader
 * thread and the discovery probe share one parser and transport_test.c can
 * drive it directly.
 */
#ifndef VCX_TRANSPORT_H
#define VCX_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#define VCX_DELIM      0xBB

/* PASSTHRU_MSG.Data is fixed at 4,128 bytes.  Size the wire buffers for the
 * worst case where every content byte needs escaping. */
#define VCX_MAX_MSG_DATA 4128
#define VCX_MAX_CONTENT  (4 + 6 + VCX_MAX_MSG_DATA + 1)
#define VCX_MAX_FRAME    (2 * VCX_MAX_CONTENT + 2)

uint8_t vcx_checksum(const uint8_t *c, int n);

/* Returns the frame length written to out, or -1 if the content exceeds
 * VCX_MAX_CONTENT or the escaped frame does not fit in cap. */
int vcx_frame_build(uint8_t cmd_hi, uint8_t opcode, uint8_t chan,
                    const uint8_t *payload, int plen, uint8_t *out, int cap);

typedef enum {
    VCX_RX_NONE,       /* byte consumed; no frame ended */
    VCX_RX_NOISE,      /* byte ahead of the first delimiter, discarded */
    VCX_RX_FRAME,      /* frame ended with a good checksum */
    VCX_RX_BAD_CSUM,   /* frame ended with a checksum mismatch */
    VCX_RX_OVERSIZE    /* frame ended after overrunning buf; dropped */
} vcx_rx_event_t;

/* After VCX_RX_FRAME or VCX_RX_BAD_CSUM, buf[0..len) is the unescaped content
 * including its trailing check byte.  After VCX_RX_OVERSIZE, len == cap.  The
 * next byte pushed may overwrite buf. */
typedef struct {
    uint8_t *buf;
    int cap;
    int n;
    int len;
    bool started, esc, over;
} vcx_parser_t;

void vcx_parser_init(vcx_parser_t *p, uint8_t *buf, int cap);
vcx_rx_event_t vcx_parser_push(vcx_parser_t *p, uint8_t b);

#endif
