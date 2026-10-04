/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/* Wire framing and receive-parser regressions. Pure bytes: no port, no clock. */
#include <stdio.h>
#include <string.h>
#include "vcx_transport.h"

static int failures;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)

typedef struct {
    int frames, bad, over, noise;
    uint8_t last[VCX_MAX_CONTENT];
    int last_len;   /* content length of the last good frame, check byte excluded */
} tally_t;

static void feed(vcx_parser_t *p, const uint8_t *b, int n, tally_t *t)
{
    for (int i = 0; i < n; i++) {
        switch (vcx_parser_push(p, b[i])) {
        case VCX_RX_FRAME:
            t->frames++; t->last_len = p->len - 1; memcpy(t->last, p->buf, (size_t)p->len);
            break;
        case VCX_RX_BAD_CSUM: t->bad++; break;
        case VCX_RX_OVERSIZE: t->over++; break;
        case VCX_RX_NOISE: t->noise++; break;
        case VCX_RX_NONE: break;
        }
    }
}

static bool built_is(uint8_t cmd_hi, uint8_t op, uint8_t chan, const uint8_t *pl, int plen,
                     const uint8_t *want, int want_len)
{
    uint8_t out[64];
    int n = vcx_frame_build(cmd_hi, op, chan, pl, plen, out, sizeof(out));
    return n == want_len && memcmp(out, want, (size_t)n) == 0;
}

static uint8_t rxbuf[VCX_MAX_CONTENT];
static uint8_t wire[VCX_MAX_FRAME];

/* Wire vectors from notes/host_protocol_map.md (link check, J1939 OPEN, CLOSE). */
static void test_known_frames(void)
{
    static const uint8_t getinfo[] = {0xBB, 0x80, 0x00, 0x8C, 0x00, 0x0C, 0xBB};
    static const uint8_t open_pl[] = {0x00, 0x00, 0x82, 0x01};
    static const uint8_t open_j1939[] = {0xBB, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00, 0x82, 0x01, 0x43, 0xBB};
    static const uint8_t close0[] = {0xBB, 0x80, 0x00, 0x41, 0x00, 0xC1, 0xBB};
    CHECK(built_is(0x00, 0x8C, 0, NULL, 0, getinfo, sizeof(getinfo)));
    CHECK(built_is(0x00, 0x40, 0, open_pl, sizeof(open_pl), open_j1939, sizeof(open_j1939)));
    CHECK(built_is(0x00, 0x41, 0, NULL, 0, close0, sizeof(close0)));
}

static void test_escaping(void)
{
    /* BB/EE/DD in the payload, then a check byte (0x3B opcode makes it BB) that
     * itself needs escaping. */
    static const uint8_t pl[] = {0xBB, 0xEE, 0xDD};
    static const uint8_t esc_pl[] = {0xBB, 0x80, 0x00, 0x00, 0x00, 0xDD, 0x44, 0xDD, 0x11, 0xDD, 0x22, 0x06, 0xBB};
    static const uint8_t esc_ck[] = {0xBB, 0x80, 0x00, 0x3B, 0x00, 0xDD, 0x44, 0xBB};
    CHECK(built_is(0x00, 0x00, 0, pl, sizeof(pl), esc_pl, sizeof(esc_pl)));
    CHECK(built_is(0x00, 0x3B, 0, NULL, 0, esc_ck, sizeof(esc_ck)));

    vcx_parser_t p; tally_t t = {0};
    vcx_parser_init(&p, rxbuf, sizeof(rxbuf));
    feed(&p, esc_pl, sizeof(esc_pl), &t);
    CHECK(t.frames == 1 && t.last_len == 7 && memcmp(&t.last[4], pl, sizeof(pl)) == 0);
    feed(&p, esc_ck, sizeof(esc_ck), &t);
    CHECK(t.frames == 2 && t.last_len == 4 && t.last[4] == 0xBB && t.bad == 0);
}

static void test_round_trip_all_bytes(void)
{
    uint8_t pl[256];
    for (int i = 0; i < 256; i++) pl[i] = (uint8_t)i;
    int n = vcx_frame_build(0x01, 0x00, 3, pl, sizeof(pl), wire, sizeof(wire));
    CHECK(n > 0);
    vcx_parser_t p; tally_t t = {0};
    vcx_parser_init(&p, rxbuf, sizeof(rxbuf));
    feed(&p, wire, n, &t);
    CHECK(t.frames == 1 && t.bad == 0 && t.over == 0 && t.noise == 0);
    CHECK(t.last_len == 4 + 256 && t.last[0] == 0x80 && t.last[1] == 0x01 && t.last[3] == 3);
    CHECK(memcmp(&t.last[4], pl, sizeof(pl)) == 0);
}

/* The reader hands the parser whatever each ReadFile returned: a frame split at
 * any byte, including between DD and its escaped byte, must still parse. */
static void test_fragmentation(void)
{
    static const uint8_t pl[] = {0xDD, 0x01, 0xBB, 0xEE};
    int n = vcx_frame_build(0x00, 0x48, 1, pl, sizeof(pl), wire, sizeof(wire));
    CHECK(n > 0);
    for (int k = 0; k <= n; k++) {
        vcx_parser_t p; tally_t t = {0};
        vcx_parser_init(&p, rxbuf, sizeof(rxbuf));
        feed(&p, wire, k, &t);
        CHECK(t.frames == 0 || k == n);
        feed(&p, wire + k, n - k, &t);
        CHECK(t.frames == 1 && t.bad == 0 && t.last_len == 4 + (int)sizeof(pl));
        CHECK(memcmp(&t.last[4], pl, sizeof(pl)) == 0);
    }
}

static void test_back_to_back(void)
{
    static const uint8_t two_shared[] = {0xBB, 0x80, 0x00, 0x8C, 0x00, 0x0C, 0xBB,
                                         0x80, 0x00, 0x41, 0x00, 0xC1, 0xBB};
    vcx_parser_t p; tally_t t = {0};
    vcx_parser_init(&p, rxbuf, sizeof(rxbuf));
    feed(&p, two_shared, sizeof(two_shared), &t);
    CHECK(t.frames == 2 && t.last[2] == 0x41);

    /* Separately built frames meet as BB BB: the empty frame between is ignored. */
    int a = vcx_frame_build(0x00, 0x8C, 0, NULL, 0, wire, sizeof(wire));
    int b = vcx_frame_build(0x00, 0x41, 0, NULL, 0, wire + a, (int)sizeof(wire) - a);
    memset(&t, 0, sizeof(t));
    feed(&p, wire, a + b, &t);
    CHECK(t.frames == 2 && t.bad == 0 && t.over == 0 && t.last[2] == 0x41);
}

static void test_noise_and_attach(void)
{
    /* Bytes before the first delimiter, including a stray escape, are noise and
     * must not leak escape state into the first frame. */
    static const uint8_t noise[] = {0x01, 0x02, 0xDD, 0x03};
    static const uint8_t getinfo[] = {0xBB, 0x80, 0x00, 0x8C, 0x00, 0x0C, 0xBB};
    vcx_parser_t p; tally_t t = {0};
    vcx_parser_init(&p, rxbuf, sizeof(rxbuf));
    feed(&p, noise, sizeof(noise), &t);
    feed(&p, getinfo, sizeof(getinfo), &t);
    CHECK(t.noise == 4 && t.frames == 1 && t.bad == 0 && t.last[2] == 0x8C);

    /* Attached mid-frame: the tail before the first BB is noise, then sync. */
    static const uint8_t tail_then_frame[] = {0x00, 0x0C, 0xBB, 0xBB, 0x80, 0x00, 0x41, 0x00, 0xC1, 0xBB};
    vcx_parser_init(&p, rxbuf, sizeof(rxbuf)); memset(&t, 0, sizeof(t));
    feed(&p, tail_then_frame, sizeof(tail_then_frame), &t);
    CHECK(t.noise == 2 && t.frames == 1 && t.bad == 0 && t.last[2] == 0x41);
}

static void test_corruption(void)
{
    static const uint8_t pl[] = {0x11, 0x22, 0x33, 0x44};
    int n = vcx_frame_build(0x00, 0x45, 0, pl, sizeof(pl), wire, sizeof(wire));
    static const uint8_t good[] = {0xBB, 0x80, 0x00, 0x41, 0x00, 0xC1, 0xBB};
    uint8_t bad[32];

    /* flipped payload bit, wrong check byte, and a byte lost on the wire */
    for (int mode = 0; mode < 3; mode++) {
        memcpy(bad, wire, (size_t)n);
        int bn = n;
        if (mode == 0) bad[5] ^= 0x01;
        else if (mode == 1) bad[n - 2] ^= 0xFF;
        else { memmove(&bad[5], &bad[6], (size_t)(n - 6)); bn--; }
        vcx_parser_t p; tally_t t = {0};
        vcx_parser_init(&p, rxbuf, sizeof(rxbuf));
        feed(&p, bad, bn, &t);
        CHECK(t.bad == 1 && t.frames == 0);
        CHECK(p.len == bn - 2);
        feed(&p, good, sizeof(good), &t);
        CHECK(t.frames == 1 && t.bad == 1 && t.last[2] == 0x41);
    }

    /* DD then BB: the delimiter wins, the frame fails its check, and the
     * pending escape does not carry into the next frame. */
    static const uint8_t dangling[] = {0xBB, 0x80, 0xDD, 0xBB, 0x80, 0x00, 0x41, 0x00, 0xC1, 0xBB};
    vcx_parser_t p; tally_t t = {0};
    vcx_parser_init(&p, rxbuf, sizeof(rxbuf));
    feed(&p, dangling, sizeof(dangling), &t);
    CHECK(t.bad == 1 && t.frames == 1 && t.last_len == 4 && t.last[0] == 0x80);
}

static void test_oversize(void)
{
    enum { CAP = 5 };
    uint8_t small[CAP];
    static const uint8_t good[] = {0xBB, 0x80, 0x00, 0x8C, 0x00, 0x0C, 0xBB};

    /* Exactly fills the buffer (4 header + check): accepted. */
    vcx_parser_t p; tally_t t = {0};
    vcx_parser_init(&p, small, CAP);
    feed(&p, good, sizeof(good), &t);
    CHECK(t.frames == 1 && t.over == 0 && t.last_len == 4);

    /* Content 80 00 8C 00 0C 55 <check>: its first five bytes happen to carry a
     * valid check.  The old discovery parser kept those five and returned a
     * frame; the shared parser drops the whole thing as oversize. */
    static const uint8_t pl[] = {0x0C, 0x55};
    int n = vcx_frame_build(0x00, 0x8C, 0, pl, sizeof(pl), wire, sizeof(wire));
    CHECK(vcx_checksum(wire + 1, 4) == wire[5]);
    memset(&t, 0, sizeof(t));
    feed(&p, wire, n, &t);
    CHECK(t.over == 1 && t.frames == 0 && t.bad == 0 && p.len == CAP);
    feed(&p, good, sizeof(good), &t);
    CHECK(t.frames == 1 && t.over == 1);

    /* Overrun on an escaped byte, then recovery. */
    static const uint8_t esc_over[] = {0xBB, 0x80, 0x00, 0x8C, 0x00, 0x0C, 0xDD, 0x44, 0xBB};
    vcx_parser_init(&p, small, CAP); memset(&t, 0, sizeof(t));
    feed(&p, esc_over, sizeof(esc_over), &t);
    feed(&p, good, sizeof(good), &t);
    CHECK(t.over == 1 && t.frames == 1 && t.bad == 0);
}

static void test_degenerate(void)
{
    /* BB BB is an empty frame: no event.  BB 00 BB is a zero-length content
     * whose check (0) matches; handle_frame then counts it as a runt. */
    static const uint8_t s[] = {0xBB, 0xBB, 0xBB, 0x00, 0xBB};
    vcx_parser_t p; tally_t t = {0};
    vcx_parser_init(&p, rxbuf, sizeof(rxbuf));
    feed(&p, s, 2, &t);
    CHECK(t.frames == 0 && t.bad == 0 && t.over == 0 && t.noise == 0);
    feed(&p, s + 2, 3, &t);
    CHECK(t.frames == 1 && t.last_len == 0);
}

static void test_build_limits(void)
{
    static uint8_t pl[VCX_MAX_CONTENT];
    int max_pl = VCX_MAX_CONTENT - 5;
    memset(pl, 0xBB, sizeof(pl));   /* worst case: every payload byte escapes */
    int n = vcx_frame_build(0x01, 0x00, 0, pl, max_pl, wire, sizeof(wire));
    CHECK(n > 0 && n <= VCX_MAX_FRAME);
    vcx_parser_t p; tally_t t = {0};
    vcx_parser_init(&p, rxbuf, sizeof(rxbuf));
    feed(&p, wire, n, &t);
    CHECK(t.frames == 1 && t.last_len == VCX_MAX_CONTENT - 1);
    CHECK(vcx_frame_build(0x01, 0x00, 0, pl, max_pl + 1, wire, sizeof(wire)) == -1);

    uint8_t out[7];
    CHECK(vcx_frame_build(0x00, 0x8C, 0, NULL, 0, out, 7) == 7);
    CHECK(vcx_frame_build(0x00, 0x8C, 0, NULL, 0, out, 6) == -1);
    CHECK(vcx_frame_build(0x00, 0x8C, 0, NULL, 0, out, 0) == -1);
    CHECK(vcx_frame_build(0x00, 0x3B, 0, NULL, 0, out, 7) == -1);   /* escaped check needs 8 */
}

int main(void)
{
    test_known_frames();
    test_escaping();
    test_round_trip_all_bytes();
    test_fragmentation();
    test_back_to_back();
    test_noise_and_attach();
    test_corruption();
    test_oversize();
    test_degenerate();
    test_build_limits();
    printf("transport tests: %s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
