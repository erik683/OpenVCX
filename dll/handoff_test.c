/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * handoff_test.c - reproduces the HDS multi-process pattern from issue #6.
 *
 * HDS closes the device in testman.exe and, ~300 ms later, launches
 * DataListClient.exe or DTCMonitor.exe expecting to find the interface free.
 * Before the cross-process handoff, the second process could not have it: the
 * first still owned the COM port across its logical close, and the newcomer
 * scanned COM1-32 twice and reported "not found".
 *
 * Usage: handoff_test.exe <dll> hold <seconds> | busy <seconds> | grab
 *   hold: PassThruOpen, PassThruClose, then idle with the link warm -- the
 *         window in which the port must be handed over on request.
 *   busy: PassThruOpen and stay open -- the holder must refuse to hand over,
 *         and the newcomer must say so promptly instead of scanning COM1-32.
 *   grab: PassThruOpen and report how long it took, then close.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <windows.h>

typedef long(WINAPI *open_fn)(void *, unsigned long *);
typedef long(WINAPI *close_fn)(unsigned long);
typedef long(WINAPI *lasterr_fn)(char *);

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s <dll> hold <s> | grab\n", argv[0]); return 2; }
    HMODULE m = LoadLibraryA(argv[1]);
    if (!m) { fprintf(stderr, "LoadLibrary failed: %lu\n", GetLastError()); return 2; }
    open_fn    pt_open  = (open_fn)GetProcAddress(m, "PassThruOpen");
    close_fn   pt_close = (close_fn)GetProcAddress(m, "PassThruClose");
    lasterr_fn pt_err   = (lasterr_fn)GetProcAddress(m, "PassThruGetLastError");
    if (!pt_open || !pt_close || !pt_err) { fprintf(stderr, "exports missing\n"); return 2; }

    unsigned long dev = 0;
    DWORD t0 = GetTickCount();
    long rc = pt_open(NULL, &dev);
    DWORD ms = GetTickCount() - t0;
    char why[256] = "";
    if (rc != 0) pt_err(why);
    printf("%s: PassThruOpen rc=%ld in %lu ms%s%s\n", argv[2], rc, ms,
           rc ? " -- " : "", why);
    fflush(stdout);
    if (rc != 0) return 1;

    if (strcmp(argv[2], "busy") == 0) {
        int secs = argc > 3 ? atoi(argv[3]) : 20;
        printf("busy: session open, holding it for %d s\n", secs);
        fflush(stdout);
        Sleep((DWORD)secs * 1000);
        pt_close(dev);
        printf("busy: PassThruClose done, exiting\n");
        return 0;
    }

    pt_close(dev);
    printf("%s: PassThruClose done (link warm)\n", argv[2]);
    fflush(stdout);

    if (strcmp(argv[2], "hold") == 0) {
        int secs = argc > 3 ? atoi(argv[3]) : 20;
        printf("hold: holding the warm link for %d s\n", secs);
        fflush(stdout);
        Sleep((DWORD)secs * 1000);
        printf("hold: exiting\n");
    }
    return 0;
}
