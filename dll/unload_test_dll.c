/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/* Actual backend and DllMain, with an offline way to start the real watcher. */
#include "device_vcx.c"
#include "api.c"

__declspec(dllexport) int test_start_watcher(void)
{
    if (!pin_worker_module()) return 0;
    s_handoff_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (!s_handoff_quit) return 0;
    s_handoff_thread = CreateThread(NULL, 0, handoff_proc, NULL, 0, NULL);
    return s_handoff_thread != NULL;
}
