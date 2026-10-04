/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * j2534.h - SAE J2534-1 (04.04) API declarations for the PC-side DLL.
 *
 * Function semantics follow the J2534-1 recommended practice; error code,
 * IOCTL, protocol and flag constants come from the firmware's shared
 * j2534_defs.h so DLL and device can never disagree.
 */
#pragma once

#include <windows.h>

#include "j2534_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned long ProtocolID;
    unsigned long RxStatus;
    unsigned long TxFlags;
    unsigned long Timestamp;
    unsigned long DataSize;
    unsigned long ExtraDataIndex;
    unsigned char Data[4128];
} PASSTHRU_MSG;

typedef struct {
    unsigned long Parameter;
    unsigned long Value;
} SCONFIG;

typedef struct {
    unsigned long NumOfParams;
    SCONFIG *ConfigPtr;
} SCONFIG_LIST;

typedef struct {
    unsigned long NumOfBytes;
    unsigned char *BytePtr;
} SBYTE_ARRAY;

long WINAPI PassThruOpen(void *pName, unsigned long *pDeviceID);
long WINAPI PassThruClose(unsigned long DeviceID);
long WINAPI PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID,
                            unsigned long Flags, unsigned long BaudRate,
                            unsigned long *pChannelID);
long WINAPI PassThruDisconnect(unsigned long ChannelID);
long WINAPI PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg,
                             unsigned long *pNumMsgs, unsigned long Timeout);
long WINAPI PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg,
                              unsigned long *pNumMsgs, unsigned long Timeout);
long WINAPI PassThruStartPeriodicMsg(unsigned long ChannelID,
                                     PASSTHRU_MSG *pMsg,
                                     unsigned long *pMsgID,
                                     unsigned long TimeInterval);
long WINAPI PassThruStopPeriodicMsg(unsigned long ChannelID,
                                    unsigned long MsgID);
long WINAPI PassThruStartMsgFilter(unsigned long ChannelID,
                                   unsigned long FilterType,
                                   PASSTHRU_MSG *pMaskMsg,
                                   PASSTHRU_MSG *pPatternMsg,
                                   PASSTHRU_MSG *pFlowControlMsg,
                                   unsigned long *pFilterID);
long WINAPI PassThruStopMsgFilter(unsigned long ChannelID,
                                  unsigned long FilterID);
long WINAPI PassThruSetProgrammingVoltage(unsigned long DeviceID,
                                          unsigned long PinNumber,
                                          unsigned long Voltage);
long WINAPI PassThruReadVersion(unsigned long DeviceID,
                                char *pFirmwareVersion, char *pDllVersion,
                                char *pApiVersion);
long WINAPI PassThruGetLastError(char *pErrorDescription);
long WINAPI PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID,
                          void *pInput, void *pOutput);

#ifdef __cplusplus
}
#endif
