/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * pt_wire.h - wire protocol command/event identifiers.
 *
 * Pure constants shared between the firmware (pt_proto) and the PC side
 * (J2534 DLL); see docs/protocol.md for payload layouts.
 */
#pragma once

/* Command types (host -> device). Responses are (type | 0x80) and carry
 * an i32 J2534 status as their first payload field. */
#define PT_CMD_PING            0x01
#define PT_CMD_VERSION         0x02
#define PT_CMD_RESET           0x03
#define PT_CMD_OPEN            0x10
#define PT_CMD_CLOSE           0x11
#define PT_CMD_CONNECT         0x12
#define PT_CMD_DISCONNECT      0x13
#define PT_CMD_WRITE_MSG       0x14
#define PT_CMD_START_FILTER    0x15
#define PT_CMD_STOP_FILTER     0x16
#define PT_CMD_START_PERIODIC  0x17
#define PT_CMD_STOP_PERIODIC   0x18
#define PT_CMD_IOCTL           0x19
#define PT_CMD_SET_PROG_VOLTAGE 0x1A

#define PT_RESP_FLAG           0x80

/* Asynchronous events (device -> host, seq = 0). */
#define PT_EVT_MSG             0xC0
#define PT_EVT_ERROR           0xC1

#define PT_PROTO_VERSION       1
