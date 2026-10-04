/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * dll_smoke.c - loads the J2534 DLL, verifies all 04.04 exports resolve
 * undecorated, and exercises the no-device and (optionally) live paths.
 *
 * Usage: dll_smoke.exe <path-to-dll> [live|exports-only]
 *   exports-only: validates loading/exports/errors without opening any COM port.
 *   live: requires a flashed esp-passthru device; runs open -> version ->
 *         CAN self-test loopback -> close.
 */
#include <stdio.h>
#include <string.h>
#include <windows.h>

typedef struct {
    unsigned long ProtocolID, RxStatus, TxFlags, Timestamp, DataSize,
        ExtraDataIndex;
    unsigned char Data[4128];
} PASSTHRU_MSG;

typedef long(WINAPI *open_fn)(void *, unsigned long *);
typedef long(WINAPI *close_fn)(unsigned long);
typedef long(WINAPI *connect_fn)(unsigned long, unsigned long, unsigned long,
                                 unsigned long, unsigned long *);
typedef long(WINAPI *disconnect_fn)(unsigned long);
typedef long(WINAPI *msgs_fn)(unsigned long, PASSTHRU_MSG *, unsigned long *,
                              unsigned long);
typedef long(WINAPI *filter_fn)(unsigned long, unsigned long, PASSTHRU_MSG *,
                                PASSTHRU_MSG *, PASSTHRU_MSG *,
                                unsigned long *);
typedef long(WINAPI *version_fn)(unsigned long, char *, char *, char *);
typedef long(WINAPI *lasterr_fn)(char *);

static const char *names[] = {
    "PassThruOpen", "PassThruClose", "PassThruConnect", "PassThruDisconnect",
    "PassThruReadMsgs", "PassThruWriteMsgs", "PassThruStartPeriodicMsg",
    "PassThruStopPeriodicMsg", "PassThruStartMsgFilter",
    "PassThruStopMsgFilter", "PassThruSetProgrammingVoltage",
    "PassThruReadVersion", "PassThruGetLastError", "PassThruIoctl",
};

static int failures;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (cond) {                                                          \
            printf("  ok: %s\n", msg);                                       \
        } else {                                                             \
            printf("  FAIL: %s\n", msg);                                     \
            failures++;                                                      \
        }                                                                    \
    } while (0)

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <dll> [live|exports-only]\n", argv[0]);
        return 2;
    }
    int live = argc > 2 && strcmp(argv[2], "live") == 0;
    int exports_only = argc > 2 && strcmp(argv[2], "exports-only") == 0;

    printf("loading %s\n", argv[1]);
    HMODULE dll = LoadLibraryA(argv[1]);
    if (!dll) {
        printf("  FAIL: LoadLibrary error %lu\n", GetLastError());
        return 1;
    }
    CHECK(dll != NULL, "DLL loads");

    FARPROC procs[14];
    for (int i = 0; i < 14; i++) {
        procs[i] = GetProcAddress(dll, names[i]);
        if (!procs[i]) {
            printf("  FAIL: missing export %s\n", names[i]);
            failures++;
        }
    }
    CHECK(failures == 0, "all 14 exports resolve undecorated");

    lasterr_fn get_last_error = (lasterr_fn)procs[12];
    char desc[256] = {0};
    CHECK(get_last_error(desc) == 0, "PassThruGetLastError returns NOERROR");
    printf("  last error text: \"%s\"\n", desc);
    CHECK(get_last_error(NULL) == 4, "NULL parameter rejected (ERR_NULL_PARAMETER)");

    open_fn pt_open = (open_fn)procs[0];
    close_fn pt_close = (close_fn)procs[1];

    if (!live && !exports_only) {
        /* Without a device, Open must fail gracefully with
         * ERR_DEVICE_NOT_CONNECTED (8) after the port scan. */
        unsigned long dev_id = 0;
        long rc = pt_open(NULL, &dev_id);
        printf("  PassThruOpen (no device expected): rc=%ld\n", rc);
        get_last_error(desc);
        printf("  last error text: \"%s\"\n", desc);
        CHECK(rc == 8 || rc == 0, "Open returns ERR_DEVICE_NOT_CONNECTED or finds a device");
        if (rc == 0) {
            printf("  note: a device responded! closing.\n");
            pt_close(dev_id);
        }
    } else if (live) {
        connect_fn pt_connect = (connect_fn)procs[2];
        disconnect_fn pt_disconnect = (disconnect_fn)procs[3];
        msgs_fn pt_read = (msgs_fn)procs[4];
        msgs_fn pt_write = (msgs_fn)procs[5];
        filter_fn pt_filter = (filter_fn)procs[8];
        version_fn pt_version = (version_fn)procs[11];

        unsigned long dev_id = 0;
        CHECK(pt_open(NULL, &dev_id) == 0, "PassThruOpen");
        char fw[80], dllv[80], api[80];
        CHECK(pt_version(dev_id, fw, dllv, api) == 0, "PassThruReadVersion");
        printf("  firmware=%s dll=%s api=%s\n", fw, dllv, api);

        unsigned long ch = 0;
        /* CAN, SELF_TEST device flag, 500 kbit/s */
        CHECK(pt_connect(dev_id, 5, 0x80000000ul, 500000, &ch) == 0,
              "PassThruConnect CAN self-test");

        PASSTHRU_MSG mask, pattern;
        memset(&mask, 0, sizeof(mask));
        memset(&pattern, 0, sizeof(pattern));
        mask.ProtocolID = pattern.ProtocolID = 5;
        mask.DataSize = pattern.DataSize = 4;
        mask.Data[2] = 0x07; mask.Data[3] = 0xFF;
        pattern.Data[2] = 0x07; pattern.Data[3] = 0xE8;
        unsigned long filter_id = 0;
        CHECK(pt_filter(ch, 1, &mask, &pattern, NULL, &filter_id) == 0,
              "PassThruStartMsgFilter PASS 0x7E8");

        PASSTHRU_MSG tx;
        memset(&tx, 0, sizeof(tx));
        tx.ProtocolID = 5;
        tx.DataSize = 9;
        tx.Data[2] = 0x07; tx.Data[3] = 0xE8;
        memcpy(&tx.Data[4], "hello", 5);
        unsigned long n = 1;
        CHECK(pt_write(ch, &tx, &n, 1000) == 0, "PassThruWriteMsgs");

        PASSTHRU_MSG rx;
        memset(&rx, 0, sizeof(rx));
        n = 1;
        long rrc = pt_read(ch, &rx, &n, 2000);
        CHECK(rrc == 0 && n == 1, "PassThruReadMsgs returns loopback frame");
        if (n == 1) {
            CHECK(rx.DataSize == 9 && memcmp(&rx.Data[4], "hello", 5) == 0,
                  "loopback payload matches");
            printf("  rx id=%02X%02X%02X%02X ts=%lu\n", rx.Data[0],
                   rx.Data[1], rx.Data[2], rx.Data[3], rx.Timestamp);
        }

        CHECK(pt_disconnect(ch) == 0, "PassThruDisconnect");
        CHECK(pt_close(dev_id) == 0, "PassThruClose");
    }

    FreeLibrary(dll);
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
