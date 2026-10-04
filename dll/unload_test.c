/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/* Run unload in a bounded child: a loader-lock regression must fail the test
 * instead of hanging the build. Neither child nor test DLL opens a COM port. */
#include <windows.h>
#include <stdio.h>
#include <string.h>

static int child(const char *path)
{
    HMODULE dll = LoadLibraryA(path);
    if (!dll || !FreeLibrary(dll)) return 2; /* unused DLL still unloads */
    dll = LoadLibraryA(path);
    if (!dll) return 3;
    FARPROC address = GetProcAddress(dll, "test_start_watcher");
    int (*start)(void) = (int (*)(void))address;
    if (!start || !start()) return 4;
    if (!FreeLibrary(dll)) return 5;
    HMODULE retained = NULL;
    if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                           GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           (LPCSTR)address, &retained) || retained != dll) return 6;
    return 0;
}

int main(int argc, char **argv)
{
    if (argc == 3 && !strcmp(argv[1], "--child")) return child(argv[2]);
    if (argc != 2) return 2;
    char exe[MAX_PATH], command[3 * MAX_PATH];
    if (!GetModuleFileNameA(NULL, exe, sizeof(exe))) return 2;
    snprintf(command, sizeof(command), "\"%s\" --child \"%s\"", exe, argv[1]);
    STARTUPINFOA si = {0}; si.cb = sizeof(si);
    PROCESS_INFORMATION pi = {0};
    if (!CreateProcessA(exe, command, NULL, NULL, FALSE, CREATE_NO_WINDOW,
                        NULL, NULL, &si, &pi)) return 2;
    DWORD result = 1;
    if (WaitForSingleObject(pi.hProcess, 5000) == WAIT_OBJECT_0)
        GetExitCodeProcess(pi.hProcess, &result);
    else {
        TerminateProcess(pi.hProcess, 1);
        WaitForSingleObject(pi.hProcess, 1000);
        puts("FAIL: FreeLibrary child did not finish within 5 seconds");
    }
    CloseHandle(pi.hThread); CloseHandle(pi.hProcess);
    printf("unload regression: %s (child exit %lu)\n", result ? "FAILED" : "PASSED", result);
    return result ? 1 : 0;
}
