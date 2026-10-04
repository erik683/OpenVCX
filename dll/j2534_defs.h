/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * j2534_defs.h - SAE J2534-1 (04.04) constants.
 *
 * Values transcribed from the J2534-1 specification. This header is pure C
 * with no platform dependencies so it can be shared by firmware and
 * host-side unit tests.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PASSTHRU_MSG.Data is a fixed 4,128-byte array in the J2534-1 spec, so no
 * message payload can exceed this.  Shared so the validator and the wire layer
 * agree on the one ceiling. */
#define J2534_MSG_DATA_MAX 4128u

/* ---- Protocol IDs (J2534-1) ---- */
#define J2534_J1850VPW          1u
#define J2534_J1850PWM          2u
#define J2534_ISO9141           3u
#define J2534_ISO14230          4u
#define J2534_CAN               5u
#define J2534_ISO15765          6u
#define J2534_SCI_A_ENGINE      7u
#define J2534_SCI_A_TRANS       8u
#define J2534_SCI_B_ENGINE      9u
#define J2534_SCI_B_TRANS       10u

/* ---- Return / error codes ---- */
#define STATUS_NOERROR              0x00
#define ERR_NOT_SUPPORTED           0x01
#define ERR_INVALID_CHANNEL_ID      0x02
#define ERR_INVALID_PROTOCOL_ID     0x03
#define ERR_NULL_PARAMETER          0x04
#define ERR_INVALID_IOCTL_VALUE     0x05
#define ERR_INVALID_FLAGS           0x06
#define ERR_FAILED                  0x07
#define ERR_DEVICE_NOT_CONNECTED    0x08
#define ERR_TIMEOUT                 0x09
#define ERR_INVALID_MSG             0x0A
#define ERR_INVALID_TIME_INTERVAL   0x0B
#define ERR_EXCEEDED_LIMIT          0x0C
#define ERR_INVALID_MSG_ID          0x0D
#define ERR_DEVICE_IN_USE           0x0E
#define ERR_INVALID_IOCTL_ID        0x0F
#define ERR_BUFFER_EMPTY            0x10
#define ERR_BUFFER_FULL             0x11
#define ERR_BUFFER_OVERFLOW         0x12
#define ERR_PIN_INVALID             0x13
#define ERR_CHANNEL_IN_USE          0x14
#define ERR_MSG_PROTOCOL_ID         0x15
#define ERR_INVALID_FILTER_ID       0x16
#define ERR_NO_FLOW_CONTROL         0x17
#define ERR_NOT_UNIQUE              0x18
#define ERR_INVALID_BAUDRATE        0x19
#define ERR_INVALID_DEVICE_ID       0x1A

/* ---- Connect flags ---- */
#define J2534_CONNECT_CAN_29BIT_ID          0x00000100u
#define J2534_CONNECT_ISO9141_NO_CHECKSUM   0x00000200u
#define J2534_CONNECT_CAN_ID_BOTH           0x00000800u
#define J2534_CONNECT_ISO9141_K_LINE_ONLY   0x00001000u
/* Device-specific connect flags (top byte reserved for this device,
 * documented in docs/protocol.md). */
#define PT_CONNECT_LISTEN_ONLY              0x40000000u
#define PT_CONNECT_SELF_TEST                0x80000000u

/* ---- TxFlags ---- */
#define J2534_TX_ISO15765_FRAME_PAD     0x00000040u
#define J2534_TX_ISO15765_ADDR_TYPE     0x00000080u
#define J2534_TX_CAN_29BIT_ID           0x00000100u
#define J2534_TX_WAIT_P3_MIN_ONLY       0x00000200u
#define J2534_TX_SCI_MODE               0x00400000u
#define J2534_TX_SCI_TX_VOLTAGE         0x00800000u

/* ---- RxStatus ---- */
#define J2534_RX_TX_MSG_TYPE            0x00000001u
#define J2534_RX_START_OF_MESSAGE       0x00000002u
#define J2534_RX_RX_BREAK               0x00000004u
#define J2534_RX_TX_INDICATION          0x00000008u
#define J2534_RX_ISO15765_PADDING_ERROR 0x00000010u
#define J2534_RX_ISO15765_ADDR_TYPE     0x00000080u
#define J2534_RX_CAN_29BIT_ID           0x00000100u

/* ---- J2534-2 extended protocol IDs (_PS = J1962 pin select) ----
 * The application picks the OBD pins for these with the J1962_PINS config
 * parameter below; that is how a Ford app asks for MS-CAN (pins 3/11)
 * instead of HS-CAN (pins 6/14) on the same connector. */
#define J2534_2_J1850VPW_PS         0x8000u
#define J2534_2_J1850PWM_PS         0x8001u
#define J2534_2_ISO9141_PS          0x8002u
#define J2534_2_ISO14230_PS         0x8003u
#define J2534_2_CAN_PS              0x8004u
#define J2534_2_ISO15765_PS         0x8005u
#define J2534_2_SW_ISO15765_PS      0x8007u
#define J2534_2_SW_CAN_PS           0x8008u
#define J2534_2_UART_ECHO_BYTE_PS   0x800Au
#define J2534_2_HONDA_DIAGH_PS      0x800Bu
#define J2534_2_TP2_0_PS            0x800Eu
#define J2534_2_FT_CAN_PS           0x800Fu
#define J2534_2_FT_ISO15765_PS      0x8010u

/* OpenVCX extensions. 0x800C/0x800D are the J2534-2 J1939_PS/J1708_PS IDs
 * mapped to raw engines (J2534-2 J1939 semantics are not implemented); the
 * 0x802x IDs are private, not SAE assignments.
 * Native config values use firmware units/semantics; see CONTRIBUTING.md. */
#define VCXID_J1939_PS 0x800Cu
#define VCXID_J1708_PS 0x800Du
#define VCXID_LIN      0x8020u
#define VCXID_TP20     0x8021u
#define VCXID_TP16     0x8022u
#define VCXID_KW82     0x8023u
#define VCX_CFG_ISO15765_EXT       0x18001u
#define VCX_CFG_ISO15765_TXBS      0x18007u
#define VCX_CFG_ISO15765_TXSTMIN   0x18008u
#define VCX_CFG_ISO15765_WAIT_MULT 0x18009u
#define VCX_CFG_ISO15765_TIMEOUT_US 0x1800Au

/* ---- J2534-2 SET_CONFIG parameters ---- */
#define J2534_2_CFG_CAN_MIXED_FORMAT 0x8000u
/* Value is 0xPPQQ: PP = J1962 pin for the primary line, QQ for the secondary.
 * 0x060E = HS-CAN (pins 6/14); 0x030B = Ford MS-CAN (pins 3/11). */
#define J2534_2_CFG_J1962_PINS       0x8001u
/* UART Echo Byte (UART_ECHO_BYTE_PS) five-baud timing, milliseconds: T0 idle
 * before the address byte, T1 address -> sync, T2 sync -> key byte 1 (Quantex's
 * summary of J2534-2 table 36). T3 is read as key byte 1 -> 2 by pattern only. */
#define J2534_2_CFG_UEB_T0_MIN       0x8028u
#define J2534_2_CFG_UEB_T1_MAX       0x8029u
#define J2534_2_CFG_UEB_T2_MAX       0x802Au
#define J2534_2_CFG_UEB_T3_MAX       0x802Bu

/* ---- Filter types ---- */
#define J2534_PASS_FILTER           1u
#define J2534_BLOCK_FILTER          2u
#define J2534_FLOW_CONTROL_FILTER   3u

/* ---- IOCTL IDs ---- */
#define J2534_IOCTL_GET_CONFIG                       0x01
#define J2534_IOCTL_SET_CONFIG                       0x02
#define J2534_IOCTL_READ_VBATT                       0x03
#define J2534_IOCTL_FIVE_BAUD_INIT                   0x04
#define J2534_IOCTL_FAST_INIT                        0x05
#define J2534_IOCTL_CLEAR_TX_BUFFER                  0x07
#define J2534_IOCTL_CLEAR_RX_BUFFER                  0x08
#define J2534_IOCTL_CLEAR_PERIODIC_MSGS              0x09
#define J2534_IOCTL_CLEAR_MSG_FILTERS                0x0A
#define J2534_IOCTL_CLEAR_FUNCT_MSG_LOOKUP_TABLE     0x0B
#define J2534_IOCTL_ADD_TO_FUNCT_MSG_LOOKUP_TABLE    0x0C
#define J2534_IOCTL_DELETE_FROM_FUNCT_MSG_LOOKUP_TBL 0x0D
#define J2534_IOCTL_READ_PROG_VOLTAGE                0x0E
/* Highest IoctlID the J2534-1/-2 documents assign.  Everything from 0x01 up to
 * here is a REAL ioctl whether or not this DLL implements it, and that
 * distinction is the whole error contract: a defined id we do not handle is
 * ERR_NOT_SUPPORTED ("valid request, this device cannot honour it"), while an id
 * outside the range is ERR_INVALID_IOCTL_ID ("that is not an ioctl").  A
 * capability probe walking the J2534-2 ioctls (SW_CAN_HS 0x8000-family,
 * READ_PIN_VOLTAGE 0x1C, ...) needs the first answer, not the second.  The
 * ceiling covers the J2534-2 channel ioctls above 0x0E. */
#define J2534_IOCTL_ID_MAX                           0x20

/* Is `id` a documented J2534-1/-2 IoctlID (as opposed to a bad argument)? */
static __inline int j2534_ioctl_id_defined(unsigned long id)
{
    return id >= J2534_IOCTL_GET_CONFIG && id <= J2534_IOCTL_ID_MAX;
}

/* ---- GET_CONFIG / SET_CONFIG parameters ---- */
#define J2534_CFG_DATA_RATE         0x01
#define J2534_CFG_LOOPBACK          0x03
#define J2534_CFG_NODE_ADDRESS      0x04
#define J2534_CFG_NETWORK_LINE      0x05
#define J2534_CFG_P1_MIN            0x06
#define J2534_CFG_P1_MAX            0x07
#define J2534_CFG_P2_MIN            0x08
#define J2534_CFG_P2_MAX            0x09
#define J2534_CFG_P3_MIN            0x0A
#define J2534_CFG_P3_MAX            0x0B
#define J2534_CFG_P4_MIN            0x0C
#define J2534_CFG_P4_MAX            0x0D
#define J2534_CFG_W1                0x0E
#define J2534_CFG_W2                0x0F
#define J2534_CFG_W3                0x10
#define J2534_CFG_W4                0x11
#define J2534_CFG_W5                0x12
#define J2534_CFG_TIDLE             0x13
#define J2534_CFG_TINIL             0x14
#define J2534_CFG_TWUP              0x15
#define J2534_CFG_PARITY            0x16
#define J2534_CFG_BIT_SAMPLE_POINT  0x17
#define J2534_CFG_SYNC_JUMP_WIDTH   0x18
#define J2534_CFG_W0                0x19
#define J2534_CFG_T1_MAX            0x1A
#define J2534_CFG_T2_MAX            0x1B
#define J2534_CFG_T4_MAX            0x1C
#define J2534_CFG_T5_MAX            0x1D
#define J2534_CFG_ISO15765_BS       0x1E
#define J2534_CFG_ISO15765_STMIN    0x1F
#define J2534_CFG_DATA_BITS         0x20
#define J2534_CFG_FIVE_BAUD_MOD     0x21
#define J2534_CFG_BS_TX             0x22
#define J2534_CFG_STMIN_TX          0x23
#define J2534_CFG_T3_MAX            0x24
#define J2534_CFG_ISO15765_WFT_MAX  0x25

#ifdef __cplusplus
}
#endif
