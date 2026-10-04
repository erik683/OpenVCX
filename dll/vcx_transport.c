/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/* vcx_transport.c - VCX Nano frame builder and receive parser; see vcx_transport.h. */
#include "vcx_transport.h"

uint8_t vcx_checksum(const uint8_t *c, int n)
{ unsigned s = 0; for (int i = 0; i < n; i++) s += c[i]; return (uint8_t)(s & 0xFF); }

int vcx_frame_build(uint8_t cmd_hi, uint8_t opcode, uint8_t chan,
                    const uint8_t *payload, int plen, uint8_t *out, int cap)
{
    uint8_t c[VCX_MAX_CONTENT]; int n = 0;
    c[n++] = 0x80; c[n++] = cmd_hi; c[n++] = opcode; c[n++] = chan;
    if (plen > (int)sizeof(c) - n - 1) return -1;
    for (int i = 0; i < plen; i++) c[n++] = payload[i];
    c[n] = vcx_checksum(c, n); n++;
    int o = 0; if (o >= cap) return -1; out[o++] = VCX_DELIM;
    for (int i = 0; i < n; i++) {
        uint8_t b = c[i];
        if (b == 0xBB || b == 0xEE || b == 0xDD) {
            if (o + 2 > cap) return -1;
            out[o++] = 0xDD;
            out[o++] = (uint8_t)(~b);
        } else {
            if (o >= cap) return -1;
            out[o++] = b;
        }
    }
    if (o >= cap) return -1;
    out[o++] = VCX_DELIM;
    return o;
}

void vcx_parser_init(vcx_parser_t *p, uint8_t *buf, int cap)
{
    p->buf = buf; p->cap = cap; p->n = 0; p->len = 0;
    p->started = false; p->esc = false; p->over = false;
}

vcx_rx_event_t vcx_parser_push(vcx_parser_t *p, uint8_t b)
{
    if (b == VCX_DELIM) {
        /* BB ends one frame and starts the next.  A frame that overran buf is
         * refused outright: with bytes missing, its checksum would pass or fail
         * at random. */
        vcx_rx_event_t ev = VCX_RX_NONE;
        p->len = p->n;
        if (p->over) ev = VCX_RX_OVERSIZE;
        else if (p->n >= 1)
            ev = vcx_checksum(p->buf, p->n - 1) == p->buf[p->n - 1] ? VCX_RX_FRAME
                                                                     : VCX_RX_BAD_CSUM;
        p->n = 0; p->started = true; p->esc = false; p->over = false;
        return ev;
    }
    if (!p->started) return VCX_RX_NOISE;
    if (p->esc) { b = (uint8_t)(~b); p->esc = false; }
    else if (b == 0xDD) { p->esc = true; return VCX_RX_NONE; }
    if (p->n < p->cap) p->buf[p->n++] = b; else p->over = true;
    return VCX_RX_NONE;
}
