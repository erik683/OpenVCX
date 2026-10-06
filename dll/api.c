/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/*
 * api.c - SAE J2534-1 (04.04) API exports for the VCX Nano backend.
 *
 * Validates caller data, translates requests for device_vcx.c, and serves its
 * receive queues through PassThruReadMsgs.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "j2534.h"
#include "device.h"
#include "pt_wire.h"
#include "pt_validate.h"
#ifdef DLL_BUILD_INFO
#include "dll_build_info.h"  /* generated per build by build_DLL_core.ps1 */
#endif
#ifdef _MSC_VER
#pragma comment(lib, "advapi32.lib")
#endif

#define DLL_VERSION_STR "0.2.0"
#define API_VERSION_STR "04.04"
#ifndef DLL_BUILD_ID
#define DLL_BUILD_ID "unfingerprinted"
#endif
#define DLL_STR2(x) #x
#define DLL_STR(x) DLL_STR2(x)
#ifdef _WIN64
#define DLL_ARCH_STR "x64"
#else
#define DLL_ARCH_STR "x86"
#endif
#if defined(_MSC_FULL_VER)
#define DLL_BUILD_TOOLCHAIN "msvc " DLL_STR(_MSC_FULL_VER) " " DLL_ARCH_STR
#elif defined(__clang__)
#define DLL_BUILD_TOOLCHAIN "clang " __clang_version__ " " DLL_ARCH_STR
#elif defined(__GNUC__)
#define DLL_BUILD_TOOLCHAIN "gcc " __VERSION__ " " DLL_ARCH_STR
#else
#define DLL_BUILD_TOOLCHAIN "unknown " DLL_ARCH_STR
#endif
#if defined(VCX_RESEARCH_CONFIG) && defined(VCX_MINIMAL_PROFILE)
#error "-ResearchConfig and -Minimal are separate builds"
#elif defined(VCX_RESEARCH_CONFIG)
#define DLL_CONFIG_STR "research"
#elif defined(VCX_MINIMAL_PROFILE)
#define DLL_CONFIG_STR "minimal"
#else
#define DLL_CONFIG_STR "release"
#endif
/* Reported as the J2534 DLL version, so it must fit the 80-byte buffer. */
#define DLL_VERSION_FULL DLL_VERSION_STR " " DLL_BUILD_ID " " DLL_CONFIG_STR
typedef char dll_version_fits[sizeof(DLL_VERSION_FULL) <= 80 ? 1 : -1];

/* VCX_CmdDevGetInfo block offsets.  These are the fields the vendor VCX.DLL
 * decodes from the same 64 bytes in logs/VCX.raw.j2534_connect_probe.exe.log
 * (HwName=VCX-NANO, FwVersion=1.9.4.2, FwDate=2023-03-31), and the version and
 * date words are the literal 02 04 09 01 00 1F 03 35 at 0x08036660 in the
 * 1.9.4.2 application image.  Other fields are not decoded here. */
#define DEVINFO_LEN      64
#define DEVINFO_NAME     28  /* up to 8 chars, NUL padded */
#define DEVINFO_VERSION  52  /* build, patch, minor, major */
#define DEVINFO_DATE     56  /* unused, day, month, years since 1970 */
#define DEVICE_ID 1
#ifndef PT_INIT_TIMEOUT_MS
#define PT_INIT_TIMEOUT_MS 3500u
#endif
/* Host/USB slack on top of the firmware's own five-baud worst case. */
#define PT_FIVE_BAUD_MARGIN_MS 500u
/* K-line ECUs sync between roughly 1200 and 10400 baud; outside this the
 * firmware's edge timing measured something that was not a sync byte. */
#define PT_FIVE_BAUD_MIN_BAUD 1000u
#define PT_FIVE_BAUD_MAX_BAUD 20000u

/* J2534-1 04.04 guarantees only an 80-byte caller buffer, including NUL. */
static char g_last_error[80] = "no error";
static SRWLOCK g_error_lock = SRWLOCK_INIT;
static LONG volatile g_open;
static char g_fw_version[80] = "unknown";

/* Format the firmware identity from a GETINFO block; "unknown" without one. */
static void format_fw_version(const uint8_t *info, uint16_t len,
                              char *out, size_t cap)
{
    char name[9] = "";
    if (len < DEVINFO_LEN) { snprintf(out, cap, "unknown"); return; }
    for (int i = 0; i < 8 && info[DEVINFO_NAME + i]; i++)
        name[i] = (info[DEVINFO_NAME + i] >= 0x20 && info[DEVINFO_NAME + i] < 0x7F)
                  ? (char)info[DEVINFO_NAME + i] : '?';
    const uint8_t *v = info + DEVINFO_VERSION, *d = info + DEVINFO_DATE;
    snprintf(out, cap, "%u.%u.%u.%u (%s %04u-%02u-%02u)", v[3], v[2], v[1], v[0],
             name, 1970u + d[3], d[2], d[1]);
}

/* SHA-256 of the module file this code was loaded from, so a log names the
 * exact binary.  It hashes the file on disk, which a replaced-after-load DLL
 * would not match. */
static void module_identity(char *path, size_t path_cap, char hex[65])
{
    static const char digits[] = "0123456789abcdef";
    HMODULE mod = NULL;
    HCRYPTPROV prov = 0;
    HCRYPTHASH hash = 0;
    HANDLE f = INVALID_HANDLE_VALUE;
    uint8_t buf[8192], digest[32];
    DWORD got = 0, dlen = sizeof(digest);
    bool ok = false;

    snprintf(path, path_cap, "unknown");
    strcpy(hex, "unavailable");
    if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                            GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                            (LPCSTR)(void *)module_identity, &mod))
        return;
    DWORD n = GetModuleFileNameA(mod, path, (DWORD)path_cap);
    if (n == 0 || n >= path_cap) { snprintf(path, path_cap, "unknown"); return; }
    f = CreateFileA(path, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_DELETE, NULL,
                    OPEN_EXISTING, 0, NULL);
    if (f == INVALID_HANDLE_VALUE ||
        !CryptAcquireContextA(&prov, NULL, NULL, PROV_RSA_AES, CRYPT_VERIFYCONTEXT) ||
        !CryptCreateHash(prov, CALG_SHA_256, 0, 0, &hash))
        goto done;
    for (;;) {
        if (!ReadFile(f, buf, sizeof(buf), &got, NULL)) goto done;
        if (got == 0) break;
        if (!CryptHashData(hash, buf, got, 0)) goto done;
    }
    ok = CryptGetHashParam(hash, HP_HASHVAL, digest, &dlen, 0) && dlen == 32;
done:
    if (ok) {
        for (int i = 0; i < 32; i++) {
            hex[2 * i] = digits[digest[i] >> 4];
            hex[2 * i + 1] = digits[digest[i] & 15];
        }
        hex[64] = 0;
    }
    if (hash) CryptDestroyHash(hash);
    if (prov) CryptReleaseContext(prov, 0);
    if (f != INVALID_HANDLE_VALUE) CloseHandle(f);
}

static long set_err(long code, const char *fmt, ...)
{
    if (code != STATUS_NOERROR) {
        char detail[240];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(detail, sizeof(detail), fmt, ap);
        va_end(ap);
        size_t n = strlen(detail);
        if (n >= sizeof(g_last_error)) n = sizeof(g_last_error) - 1;
        AcquireSRWLockExclusive(&g_error_lock);
        memcpy(g_last_error, detail, n);
        g_last_error[n] = 0;
        ReleaseSRWLockExclusive(&g_error_lock);
        dev_log("error %ld: %s", code, detail);
    }
    return code;
}

/* Transport failure (no/garbled response) maps to a J2534 error. */
static long xfer_err(const char *call)
{
    if (!dev_is_connected()) {
        return set_err(ERR_DEVICE_NOT_CONNECTED, "%s: device disconnected",
                       call);
    }
    return set_err(ERR_TIMEOUT, "%s: no response from device", call);
}

/* Log a message as the application sees it: RxStatus decides whether it is a
 * loopback echo (0x01), a TxDone indication (0x09) or bus traffic, and the
 * leading bytes identify the CAN id and service.
 *
 * The dump is whole.  It used to stop at 16 bytes with no marker, which is
 * fine for a 22-service poll and actively misleading for anything longer --
 * a multi-frame ISO15765 reply came out looking like a complete short one.
 * ts= is the low 32 bits of the host QueryPerformanceCounter clock in
 * microseconds, taken when the serial read carrying the frame returned. It is
 * a host arrival time, not a hardware timestamp.
 * is_tx selects TxFlags (outgoing) vs
 * RxStatus/Timestamp (incoming) as the second field.
 *
 * len is the already-validated byte count. Keep logging bounded independently
 * of the caller-supplied DataSize. */
static void log_pt_msg(const char *tag, unsigned long chan,
                       const PASSTHRU_MSG *m, unsigned long len, bool is_tx)
{
    if (len > sizeof(m->Data)) len = sizeof(m->Data);   /* backstop, not the length */
    /* Report the application's own DataSize whenever it disagrees with what
     * was validated, so a caller that declares a length it did not fill
     * leaves a trace instead of having its claim silently replaced. */
    char declared[32] = "";
    if (m->DataSize != len)
        snprintf(declared, sizeof(declared), " declared=%lu", m->DataSize);
    if (is_tx) {
        dev_log_hex(m->Data, (int)len, "%s ch=%lu txflags=0x%08lX len=%lu%s",
                    tag, chan, m->TxFlags, len, declared);
    } else {
        dev_log_hex(m->Data, (int)len, "%s ch=%lu ts=%lu rxstatus=0x%08lX len=%lu%s",
                    tag, chan, m->Timestamp, m->RxStatus, len, declared);
    }
}

static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

/* Decoding of the HDS repeat IOCTL ABI.
 * Snapshot through ReadProcessMemory so logging cannot fault on an input
 * which the previously unhandled IOCTL never dereferenced. Never read output. */
typedef struct {
    uint32_t interval, condition;
    PASSTHRU_MSG tx, mask, pattern;
} repeat_log_input;
static volatile LONG repeat_log_sequence;

static void log_repeat_ioctl(unsigned long ch, unsigned long id, const void *input,
                             const void *output)
{
    if (!dev_log_enabled()) return;
    unsigned long seq = (unsigned long)InterlockedIncrement(&repeat_log_sequence);
    const char *name = id == 0x8004 ? "START" : id == 0x8005 ? "QUERY" : "STOP";
    dev_log("repeat capture=%lu ch=%lu operation=%s ioctl=0x%04lX input=%p output=%p",
            seq, ch, name, id, input, output);
    SIZE_T copied = 0;
    if (id != 0x8004) {
        uint32_t message_id;
        if (!input || !ReadProcessMemory(GetCurrentProcess(), input, &message_id,
                                         sizeof(message_id), &copied) || copied != sizeof(message_id)) {
            dev_log("repeat capture=%lu message_id=unreadable", seq);
        } else {
            dev_log("repeat capture=%lu message_id=%lu", seq, (unsigned long)message_id);
        }
        return;
    }
    repeat_log_input snapshot;
    if (!input || !ReadProcessMemory(GetCurrentProcess(), input, &snapshot,
                                     sizeof(snapshot), &copied) || copied != sizeof(snapshot)) {
        dev_log("repeat capture=%lu descriptor=unreadable bytes=%llu expected=%llu", seq,
                (unsigned long long)copied, (unsigned long long)sizeof(snapshot));
        return;
    }
    dev_log("repeat capture=%lu interval=%lu condition=%lu", seq,
            (unsigned long)snapshot.interval, (unsigned long)snapshot.condition);
    const PASSTHRU_MSG *messages[] = {&snapshot.tx, &snapshot.mask, &snapshot.pattern};
    const char *roles[] = {"tx", "mask", "pattern"};
    for (unsigned k = 0; k < 3; ++k) {
        const PASSTHRU_MSG *m = messages[k];
        unsigned long len = m->DataSize;
        if (len > sizeof(m->Data)) len = sizeof(m->Data);
        dev_log("repeat capture=%lu role=%s protocol=%lu txflags=0x%08lX rxstatus=0x%08lX timestamp=%lu declared=%lu captured=%lu extra=%lu",
                seq, roles[k], m->ProtocolID, m->TxFlags, m->RxStatus, m->Timestamp,
                m->DataSize, len, m->ExtraDataIndex);
        /* Small offset-labelled chunks deliberately bypass hex_max: a mask
         * cut at 256 bytes would make this diagnostic capture incomplete. */
        for (unsigned long offset = 0; offset < len; offset += 32) {
            char hex[32 * 3 + 1];
            unsigned long n = len - offset;
            if (n > 32) n = 32;
            for (unsigned long j = 0; j < n; ++j)
                snprintf(hex + j * 3, sizeof(hex) - j * 3, "%02X%s",
                         m->Data[offset + j], j + 1 == n ? "" : " ");
            dev_log("repeat capture=%lu role=%s offset=%lu bytes=%lu: %s",
                    seq, roles[k], offset, n, hex);
        }
    }
}

static void wr_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

/* VCXPT32 implements the UART-init IOCTLs by sending an ordinary protocol
 * message with vendor-private PDU flag 0x08 (five-baud) or 0x04 (fast init).
 * The target engine interprets those flags before its normal transmit path;
 * see the ISO14230 write routine at firmware 0x08030DD6 and the vendor-DLL
 * helpers at 0x10009510 / 0x10009310. */
static long send_uart_init(unsigned long channel, uint32_t init_flag,
                           const uint8_t *data, uint16_t len)
{
#ifdef VCX_INIT_TEST
    return init_test_send(channel, init_flag, data, len);
#endif
    uint8_t req[14 + 4128];
    wr_u32(req, channel);
    wr_u32(req + 4, 0);
    wr_u32(req + 8, init_flag);
    wr_u16(req + 12, len);
    if (len) memcpy(req + 14, data, len);
    return dev_request(PT_CMD_WRITE_MSG, req, (uint16_t)(14 + len),
                       NULL, NULL, 0);
}

/* Wait long enough for the firmware to finish or give up on its own: a fixed
 * 3.5 s cut the reply off whenever an app widened W1/W5 (HDS's EPS timing
 * needs about 3.7 s). The offline harnesses keep their short fixed timeout. */
static DWORD five_baud_timeout(uint32_t worst_ms)
{
#ifdef VCX_INIT_TEST
    (void)worst_ms;
    return PT_INIT_TIMEOUT_MS;
#else
    DWORD t = worst_ms + PT_FIVE_BAUD_MARGIN_MS;
    return t > PT_INIT_TIMEOUT_MS ? t : PT_INIT_TIMEOUT_MS;
#endif
}

/* Firmware 1.9.4.2 slow-init callbacks 0803256A / 08030AAE enqueue
 * exactly: 55 key1 key2 baud_hi baud_lo 00. This is not a diagnostic frame.
 * Fast-init accepts OEM-specific payloads with at least a three-byte frame:
 * no fixed service ID or checksum assumption (HDS uses private formats). */
static bool slow_init_record(const rx_msg_t *m)
{
    return m->len == 6 && m->data[0] == 0x55 && m->data[5] == 0;
}

static bool init_reply_valid(const rx_msg_t *m, bool five_baud)
{
    if (m->rx_status & (J2534_RX_TX_MSG_TYPE | J2534_RX_TX_INDICATION |
                       J2534_RX_START_OF_MESSAGE | J2534_RX_RX_BREAK |
                       J2534_RX_ISO15765_PADDING_ERROR)) return false;
    if (five_baud) return slow_init_record(m);
    return m->len >= 3 && m->len <= J2534_MSG_DATA_MAX && !slow_init_record(m);
}

/* Wait through TX echoes and malformed events, but never extend the deadline.
 * A generation check prevents a disconnected/reused slot supplying the reply.
 * A frame stamped less than min_age_ms after sent_at_us predates the end of the
 * wake-up pattern, so it cannot answer this init (a late reply to an earlier
 * request, still in transit when the queue was cleared). */
static rx_msg_t *wait_uart_init_reply(unsigned long channel, uint64_t generation,
                                     DWORD timeout, bool five_baud, uint32_t sent_at_us,
                                     DWORD min_age_ms, long *status)
{
    DWORD start = GetTickCount();
    bool malformed = false;
    for (;;) {
        rx_msg_t *m = NULL;
        HANDLE event = NULL;
        if (!dev_channel_pop_generation(channel, generation, &m, &event)) {
            *status = ERR_INVALID_CHANNEL_ID;
            return NULL;
        }
        bool had_message = m != NULL;
        if (m && min_age_ms && (int32_t)(m->timestamp - sent_at_us) < (int32_t)(min_age_ms * 1000u) &&
            init_reply_valid(m, five_baud)) {
            dev_log("  init RX discarded: len=%u arrived %ld ms after the request, "
                    "before the wake-up pattern could end (%lu ms)", m->len,
                    (long)((int32_t)(m->timestamp - sent_at_us) / 1000), (unsigned long)min_age_ms);
            free(m);
            m = NULL;
        }
        if (m) {
            if (init_reply_valid(m, five_baud)) { *status = STATUS_NOERROR; return m; }
            if (!(m->rx_status & (J2534_RX_TX_MSG_TYPE | J2534_RX_TX_INDICATION))) malformed = true;
            dev_log("  init RX discarded: status=0x%lX len=%u (%s)",
                    (unsigned long)m->rx_status, m->len, five_baud ? "five-baud" : "fast");
            free(m);
        }
        DWORD elapsed = GetTickCount() - start;
        if (elapsed >= timeout) break;
        if (!had_message && WaitForSingleObject(event, timeout - elapsed) != WAIT_OBJECT_0) break;
    }
    *status = malformed ? ERR_INVALID_MSG : ERR_TIMEOUT;
    return NULL;
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ---- device session ---- */

static long api_PassThruOpen(void *pName, unsigned long *pDeviceID)
{
    (void)pName;
    dev_log("PassThruOpen");
    if (!pDeviceID) {
        return set_err(ERR_NULL_PARAMETER, "PassThruOpen: NULL pDeviceID");
    }
    if (g_open) {
        return set_err(ERR_DEVICE_IN_USE, "PassThruOpen: already open");
    }
    char why[160]; bool busy = false;
    if (!dev_connect(why, sizeof(why), &busy)) {
        /* "Not connected" and "somebody else has it" are different problems
         * with different fixes, and reporting the second as the first sends
         * the user hunting for a cable fault. */
        return set_err(busy ? ERR_DEVICE_IN_USE
                             : ERR_DEVICE_NOT_CONNECTED,
                       "PassThruOpen: %s", why);
    }
    /* Clear any session a crashed app left behind, then open. */
    dev_request(PT_CMD_RESET, NULL, 0, NULL, NULL, 0);
    long rc = dev_request(PT_CMD_OPEN, NULL, 0, NULL, NULL, 0);
    if (rc < 0) {
        long error = xfer_err("PassThruOpen");
        dev_disconnect();
        return error;
    }
    if (rc != STATUS_NOERROR) {
        dev_disconnect();
        return set_err(rc, "PassThruOpen: device returned %ld", rc);
    }

    uint8_t info[DEVINFO_LEN];
    uint16_t info_len = 0;
    if (dev_request(PT_CMD_VERSION, NULL, 0, info, &info_len,
                    sizeof(info)) != STATUS_NOERROR)
        info_len = 0;
    format_fw_version(info, info_len, g_fw_version, sizeof(g_fw_version));

    InterlockedExchange(&g_open, 1);
    *pDeviceID = DEVICE_ID;
    char module_path[MAX_PATH], module_hash[65];
    module_identity(module_path, sizeof(module_path), module_hash);
    dev_log("PassThruOpen -> device=%d dll=%s api=%s firmware=%s", DEVICE_ID,
            DLL_VERSION_FULL, API_VERSION_STR, g_fw_version);
    dev_log("  DLL module: %s sha256=%s toolchain=%s", module_path, module_hash,
            DLL_BUILD_TOOLCHAIN);
    return STATUS_NOERROR;
}

static long api_PassThruClose(unsigned long DeviceID)
{
    dev_log("PassThruClose");
    if (!g_open || DeviceID != DEVICE_ID) {
        return set_err(ERR_INVALID_DEVICE_ID, "PassThruClose: bad device id");
    }
    dev_request(PT_CMD_CLOSE, NULL, 0, NULL, NULL, 0);
    long rc = dev_session_close();
    if (rc < 0) {
        /* A physically lost link cannot be cleaned up through this session.
         * Release the API handle so Open can rediscover it; the backend keeps
         * uncertain voltage/timer state for cleanup on the next connection. */
        if (!dev_is_connected()) InterlockedExchange(&g_open, 0);
        return xfer_err("PassThruClose");
    }
    if (rc != STATUS_NOERROR)
        return set_err(rc, "PassThruClose: device cleanup failed (%ld)", rc);
    InterlockedExchange(&g_open, 0);
    return STATUS_NOERROR;
}

/* ---- channels ---- */

static long api_PassThruConnect(unsigned long DeviceID, unsigned long ProtocolID,
                            unsigned long Flags, unsigned long BaudRate,
                            unsigned long *pChannelID)
{
    dev_log("PassThruConnect proto=%lu flags=0x%lX baud=%lu", ProtocolID,
            Flags, BaudRate);
    if (!pChannelID) {
        return set_err(ERR_NULL_PARAMETER, "PassThruConnect: NULL pChannelID");
    }
    if (!g_open || DeviceID != DEVICE_ID) {
        return set_err(ERR_INVALID_DEVICE_ID, "PassThruConnect: bad device id");
    }
    bool strict = dev_strict_validation();
    long fv = pt_validate_connect_flags(ProtocolID, Flags, strict);
    if (fv != STATUS_NOERROR)
        return set_err(fv, "PassThruConnect: illegal flags 0x%lX for protocol %lu",
                       Flags, ProtocolID);
    long bv = pt_validate_baudrate(ProtocolID, BaudRate, strict);
    if (bv != STATUS_NOERROR)
        return set_err(bv, "PassThruConnect: invalid baud %lu for protocol %lu",
                       BaudRate, ProtocolID);
    uint8_t req[16], resp[8];
    uint16_t resp_len = 0;
    wr_u32(&req[0], ProtocolID);
    wr_u32(&req[4], Flags);
    wr_u32(&req[8], BaudRate);
    wr_u32(&req[12], 0); /* bus 0; J2534-2 multi-bus mapping comes later */
    long rc = dev_request(PT_CMD_CONNECT, req, sizeof(req), resp, &resp_len,
                          sizeof(resp));
    if (rc < 0) {
        return xfer_err("PassThruConnect");
    }
    if (rc != STATUS_NOERROR) {
        return set_err(rc, "PassThruConnect: device returned %ld", rc);
    }
    if (resp_len < 4) {
        return set_err(ERR_FAILED, "PassThruConnect: short response");
    }
    uint32_t wire_id = rd_u32(resp);
    if (!dev_channel_add(wire_id, ProtocolID)) {
        return set_err(ERR_FAILED, "PassThruConnect: channel table full");
    }
    *pChannelID = wire_id;
    /* Every later "ch=" in the log refers to this id; without it the mapping
     * from a protocol to its channel had to be inferred. */
    dev_log("PassThruConnect -> ch=%lu (proto=%lu baud=%lu)",
            (unsigned long)wire_id, ProtocolID, BaudRate);
    return STATUS_NOERROR;
}

static long api_PassThruDisconnect(unsigned long ChannelID)
{
    dev_log("PassThruDisconnect ch=%lu", ChannelID);
    if (!dev_channel_find(ChannelID)) {
        return set_err(ERR_INVALID_CHANNEL_ID,
                       "PassThruDisconnect: bad channel");
    }
    uint8_t req[4];
    wr_u32(req, ChannelID);
    long rc = dev_request(PT_CMD_DISCONNECT, req, sizeof(req), NULL, NULL, 0);
    if (rc < 0) {
        return xfer_err("PassThruDisconnect");
    }
    return rc == STATUS_NOERROR
           ? STATUS_NOERROR
           : set_err(rc, "PassThruDisconnect: device returned %ld", rc);
}

/* ---- messages ---- */

long WINAPI PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg,
                             unsigned long *pNumMsgs, unsigned long Timeout)
{
    /* What the application asked for is as much of the record as what it got:
     * a non-blocking poll loop and a 100 ms blocking read produce the same
     * "<-" lines but are completely different behaviour to match against the
     * vendor DLL.  Level 2 (the default) keeps them; level 1 drops these two
     * lines when a long capture needs to be smaller. */
    if (dev_log_verbose())
        dev_log("PassThruReadMsgs -> ch=%lu want=%lu timeout=%lu", ChannelID,
                pNumMsgs ? *pNumMsgs : 0, Timeout);
    if (!pMsg || !pNumMsgs) {
        return set_err(ERR_NULL_PARAMETER, "PassThruReadMsgs: NULL parameter");
    }
    dev_channel_snapshot_t snapshot;
    if (!dev_channel_snapshot(ChannelID, &snapshot)) {
        *pNumMsgs = 0;
        return set_err(ERR_INVALID_CHANNEL_ID, "PassThruReadMsgs: bad channel");
    }
    unsigned long want = *pNumMsgs;
    unsigned long got = 0;
    DWORD start = GetTickCount();

    /* Overflow is reported AFTER draining what is queued (see below), so a lost
     * frame does not swallow the messages that did arrive. */

    while (got < want) {
        /* A concurrent Disconnect can recycle the slot while this caller is
         * waiting.  Never deliver that new channel's traffic to this one. */
        if (got && Timeout && GetTickCount() - start >= Timeout) break;
        rx_msg_t *m = NULL;
        HANDLE event = snapshot.rx_event;
        if (!dev_channel_pop_generation(ChannelID, snapshot.generation, &m, &event)) {
            *pNumMsgs = got;
            return set_err(ERR_INVALID_CHANNEL_ID,
                           "PassThruReadMsgs: channel disconnected");
        }
        if (m) {
            PASSTHRU_MSG *out = &pMsg[got++];
            memset(out, 0, sizeof(*out) - sizeof(out->Data));
            out->ProtocolID = snapshot.protocol;
            out->RxStatus = m->rx_status;
            out->Timestamp = m->timestamp;
            out->DataSize = m->len;
            out->ExtraDataIndex = (m->rx_status & (J2534_RX_TX_INDICATION |
                                   J2534_RX_START_OF_MESSAGE)) ? 0 : m->len;
            memcpy(out->Data, m->data, m->len);
            free(m);
            if (dev_log_enabled()) {
                log_pt_msg("PassThruReadMsgs <-", ChannelID, out, out->DataSize, false);
            }
            continue;
        }
        if (Timeout == 0) {
            break;
        }
        DWORD elapsed = GetTickCount() - start;
        if (elapsed >= Timeout) {
            break;
        }
        DWORD wait = WaitForSingleObject(event, Timeout - elapsed);
        if (wait == WAIT_TIMEOUT) break;
        if (wait != WAIT_OBJECT_0) {
            *pNumMsgs = got;
            return set_err(ERR_FAILED, "PassThruReadMsgs: receive wait failed");
        }
    }

    *pNumMsgs = got;

    /* A pending buffer overflow is surfaced only after the queued messages have
     * been handed back: the frames that arrived are delivered (pNumMsgs already
     * holds their count), the lost-frame signal follows, and the flag is
     * cleared.  Re-read the channel: the drain above may itself have crossed the
     * overflow point. */
    bool overflow = false;
    if (!dev_channel_take_overflow(ChannelID, snapshot.generation, &overflow))
        return set_err(ERR_INVALID_CHANNEL_ID, "PassThruReadMsgs: channel disconnected");
    if (overflow) {
        return set_err(ERR_BUFFER_OVERFLOW,
                       "PassThruReadMsgs: receive queue overflowed (delivered %lu)",
                       got);
    }
    if (got == want) {
        if (dev_log_verbose())
            dev_log("PassThruReadMsgs <= ch=%lu got=%lu of %lu in %lu ms rc=0",
                    ChannelID, got, want,
                    (unsigned long)(GetTickCount() - start));
        return STATUS_NOERROR;
    }
    if (got == 0) {
        return set_err(ERR_BUFFER_EMPTY, "PassThruReadMsgs: no messages "
                       "(ch=%lu timeout=%lu after %lu ms)", ChannelID, Timeout,
                       (unsigned long)(GetTickCount() - start));
    }
    if (Timeout == 0) {
        /* Non-blocking read: a partial fill is success, not a timeout -- the
         * caller asked for whatever was immediately available (J2534-1 04.04). */
        if (dev_log_verbose())
            dev_log("PassThruReadMsgs <= ch=%lu got=%lu of %lu (non-blocking) rc=0",
                    ChannelID, got, want);
        return STATUS_NOERROR;
    }
    return set_err(ERR_TIMEOUT, "PassThruReadMsgs: %lu of %lu before timeout "
                   "(ch=%lu timeout=%lu after %lu ms)", got, want, ChannelID,
                   Timeout, (unsigned long)(GetTickCount() - start));
}

static long api_PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg,
                              unsigned long *pNumMsgs, unsigned long Timeout)
{
    /* Without this line an application write and a periodic- or init-generated
     * one are indistinguishable in the log: both surface only as "> TXMSG"
     * from the transport.  Gated like PassThruReadMsgs's entry/exit lines:
     * level 1 omits both high-frequency entry traces, level 2 keeps them. */
    if (dev_log_verbose())
        dev_log("PassThruWriteMsgs -> ch=%lu count=%lu timeout=%lu", ChannelID,
                pNumMsgs ? *pNumMsgs : 0, Timeout);
    if (!pMsg || !pNumMsgs) {
        return set_err(ERR_NULL_PARAMETER, "PassThruWriteMsgs: NULL parameter");
    }
    dev_channel_t *ch = dev_channel_find(ChannelID);
    if (!ch) {
        *pNumMsgs = 0;
        return set_err(ERR_INVALID_CHANNEL_ID,
                       "PassThruWriteMsgs: bad channel");
    }
    unsigned long want = *pNumMsgs;
    uint8_t *req = malloc(14 + 4128);
    if (!req) {
        *pNumMsgs = 0;
        return set_err(ERR_FAILED, "PassThruWriteMsgs: out of memory");
    }
    long rc = STATUS_NOERROR;
    DWORD start = GetTickCount();

    for (unsigned long i = 0; i < want; i++) {
        if (i && Timeout && GetTickCount() - start >= Timeout) {
            *pNumMsgs = i;
            rc = set_err(ERR_TIMEOUT, "PassThruWriteMsgs: deadline after %lu of %lu messages", i, want);
            break;
        }
        PASSTHRU_MSG *m = &pMsg[i];
        if (m->ProtocolID != ch->protocol) {
            *pNumMsgs = i;
            rc = set_err(ERR_MSG_PROTOCOL_ID,
                         "PassThruWriteMsgs: msg protocol %lu on channel "
                         "protocol %lu", m->ProtocolID, ch->protocol);
            break;
        }
        long mv = pt_validate_msg(ch->protocol, PT_OP_WRITE, m->DataSize,
                                  m->TxFlags, dev_strict_validation());
        if (mv != STATUS_NOERROR) {
            *pNumMsgs = i;
            rc = set_err(mv, "PassThruWriteMsgs: bad message (proto=%lu size=%lu)",
                         ch->protocol, m->DataSize);
            break;
        }
        if (dev_log_enabled()) log_pt_msg("  write msg", ChannelID, m, m->DataSize, true);
        DWORD elapsed = GetTickCount() - start;
        if (Timeout && elapsed >= Timeout) {
            *pNumMsgs = i;
            rc = set_err(ERR_TIMEOUT, "PassThruWriteMsgs: deadline before message %lu", i);
            break;
        }
        wr_u32(&req[0], ChannelID);
        wr_u32(&req[4], Timeout ? Timeout - elapsed : 0);
        wr_u32(&req[8], m->TxFlags);
        wr_u16(&req[12], (uint16_t)m->DataSize);
        memcpy(&req[14], m->Data, m->DataSize);
        long wrc = dev_request(PT_CMD_WRITE_MSG, req,
                               (uint16_t)(14 + m->DataSize), NULL, NULL, 0);
        if (wrc < 0) {
            *pNumMsgs = i;
            rc = xfer_err("PassThruWriteMsgs");
            break;
        }
        if (wrc != STATUS_NOERROR) {
            *pNumMsgs = i;
            rc = set_err(wrc, "PassThruWriteMsgs: device returned %ld", wrc);
            break;
        }
    }
    free(req);
    return rc;
}

/* ---- periodic messages ---- */

static long api_PassThruStartPeriodicMsg(unsigned long ChannelID,
                                     PASSTHRU_MSG *pMsg,
                                     unsigned long *pMsgID,
                                     unsigned long TimeInterval)
{
    dev_log("PassThruStartPeriodicMsg ch=%lu interval=%lu", ChannelID,
            TimeInterval);
    if (!pMsg || !pMsgID) {
        return set_err(ERR_NULL_PARAMETER,
                       "PassThruStartPeriodicMsg: NULL parameter");
    }
    dev_channel_t *pch = dev_channel_find(ChannelID);
    if (!pch) {
        return set_err(ERR_INVALID_CHANNEL_ID,
                       "PassThruStartPeriodicMsg: bad channel");
    }
    /* HDS's SRS keepalive starts a HONDA_DIAGH_PS (0x800B) message on the ISO9141
     * channel it opened for the same ECU.  Both run on firmware engine 0x9104,
     * so in vendor-compatible mode accept exactly that pair when the channel
     * already drives pin 7 (HONDA_DIAGH's K-line); strict mode still rejects. */
    bool honda_alias = !dev_strict_validation() &&
                       pMsg->ProtocolID == J2534_2_HONDA_DIAGH_PS &&
                       pch->protocol == J2534_ISO9141 &&
                       dev_channel_uart_pin(pch->wire_id) == 7;
    if (honda_alias)
        dev_log("PassThruStartPeriodicMsg: accepting HONDA_DIAGH_PS message on "
                "ISO9141 channel (same engine, pin 7)");
    else if (pMsg->ProtocolID != pch->protocol)
        return set_err(ERR_MSG_PROTOCOL_ID, "PassThruStartPeriodicMsg: protocol mismatch "
                       "(msg=%lu channel=%lu)", pMsg->ProtocolID, (unsigned long)pch->protocol);
    long mv = pt_validate_msg(pch->protocol, PT_OP_PERIODIC, pMsg->DataSize,
                              pMsg->TxFlags, dev_strict_validation());
    if (mv != STATUS_NOERROR) {
        return set_err(mv, "PassThruStartPeriodicMsg: bad message (size=%lu)",
                       pMsg->DataSize);
    }
    uint8_t req[14 + 12], resp[8];
    uint16_t resp_len = 0;
    wr_u32(&req[0], ChannelID);
    wr_u32(&req[4], TimeInterval);
    wr_u32(&req[8], pMsg->TxFlags);
    wr_u16(&req[12], (uint16_t)pMsg->DataSize);
    memcpy(&req[14], pMsg->Data, pMsg->DataSize);
    long rc = dev_request(PT_CMD_START_PERIODIC, req,
                          (uint16_t)(14 + pMsg->DataSize), resp, &resp_len,
                          sizeof(resp));
    if (rc < 0) {
        return xfer_err("PassThruStartPeriodicMsg");
    }
    if (rc != STATUS_NOERROR) {
        return set_err(rc, "PassThruStartPeriodicMsg: device returned %ld", rc);
    }
    if (resp_len != 4) return set_err(ERR_FAILED, "PassThruStartPeriodicMsg: malformed reply");
    *pMsgID = rd_u32(resp);
    dev_log("PassThruStartPeriodicMsg -> ch=%lu id=%lu interval=%lu",
            ChannelID, *pMsgID, TimeInterval);
    return STATUS_NOERROR;
}

static long api_PassThruStopPeriodicMsg(unsigned long ChannelID,
                                    unsigned long MsgID)
{
    dev_log("PassThruStopPeriodicMsg ch=%lu id=%lu", ChannelID, MsgID);
    uint8_t req[8];
    wr_u32(&req[0], ChannelID);
    wr_u32(&req[4], MsgID);
    long rc = dev_request(PT_CMD_STOP_PERIODIC, req, sizeof(req), NULL, NULL,
                          0);
    if (rc < 0) {
        return xfer_err("PassThruStopPeriodicMsg");
    }
    return rc == STATUS_NOERROR
           ? STATUS_NOERROR
           : set_err(rc, "PassThruStopPeriodicMsg: device returned %ld", rc);
}

/* ---- filters ---- */

static long api_PassThruStartMsgFilter(unsigned long ChannelID,
                                   unsigned long FilterType,
                                   PASSTHRU_MSG *pMaskMsg,
                                   PASSTHRU_MSG *pPatternMsg,
                                   PASSTHRU_MSG *pFlowControlMsg,
                                   unsigned long *pFilterID)
{
    dev_log("PassThruStartMsgFilter ch=%lu type=%lu", ChannelID, FilterType);
    if (!pMaskMsg || !pPatternMsg || !pFilterID) {
        return set_err(ERR_NULL_PARAMETER,
                       "PassThruStartMsgFilter: NULL parameter");
    }
    if (FilterType == J2534_FLOW_CONTROL_FILTER && !pFlowControlMsg) {
        return set_err(ERR_NULL_PARAMETER,
                       "PassThruStartMsgFilter: flow control filter without "
                       "flow control message");
    }
    /* The unused flow-control argument is not part of PASS/BLOCK records.
     * Its length is not validated by pt_validate_filter for those types, so
     * copying it would overflow req even with otherwise valid input. */
    if (FilterType != J2534_FLOW_CONTROL_FILTER) pFlowControlMsg = NULL;
    dev_channel_t *fch = dev_channel_find(ChannelID);
    if (!fch) {
        return set_err(ERR_INVALID_CHANNEL_ID,
                       "PassThruStartMsgFilter: bad channel");
    }
    if (pMaskMsg->ProtocolID != fch->protocol ||
        pPatternMsg->ProtocolID != fch->protocol ||
        (pFlowControlMsg && pFlowControlMsg->ProtocolID != fch->protocol))
        return set_err(ERR_MSG_PROTOCOL_ID, "PassThruStartMsgFilter: protocol mismatch "
                       "(mask=%lu pattern=%lu flow=%lu channel=%lu)", pMaskMsg->ProtocolID,
                       pPatternMsg->ProtocolID, pFlowControlMsg ? pFlowControlMsg->ProtocolID : 0,
                       (unsigned long)fch->protocol);
    /* Validate against the full unsigned DataSize BEFORE the 16-bit casts below,
     * closing the truncation hazard: a 0x1000C length would otherwise wrap to 12
     * and pass a check it should fail. */
    long fv = pt_validate_filter(fch->protocol, FilterType, pMaskMsg->DataSize,
                                 pPatternMsg->DataSize,
                                 pFlowControlMsg ? pFlowControlMsg->DataSize : 0,
                                 dev_strict_validation());
    if (fv != STATUS_NOERROR)
        return set_err(fv, "PassThruStartMsgFilter: invalid filter (type=%lu)",
                       FilterType);
    uint16_t mlen = (uint16_t)pMaskMsg->DataSize;
    uint16_t plen = (uint16_t)pPatternMsg->DataSize;
    uint16_t fclen = pFlowControlMsg ? (uint16_t)pFlowControlMsg->DataSize : 0;
    uint32_t flags = pMaskMsg->TxFlags | pPatternMsg->TxFlags |
                     (pFlowControlMsg ? pFlowControlMsg->TxFlags : 0);
    /* The filter the application actually installed, in its own terms.  These
     * bytes were only ever visible inside the wire frame, which is a different
     * layout and needed decoding by hand to check a filter against the traffic
     * it did or did not let through. */
    if (dev_log_enabled()) {
        log_pt_msg("  filter mask", ChannelID, pMaskMsg, mlen, true);
        log_pt_msg("  filter pattern", ChannelID, pPatternMsg, plen, true);
        if (pFlowControlMsg)
            log_pt_msg("  filter flowcontrol", ChannelID, pFlowControlMsg, fclen, true);
    }

    uint8_t req[18 + 36], resp[8];
    uint16_t resp_len = 0;
    wr_u32(&req[0], ChannelID);
    wr_u32(&req[4], FilterType);
    wr_u32(&req[8], flags);
    wr_u16(&req[12], mlen);
    wr_u16(&req[14], plen);
    wr_u16(&req[16], fclen);
    uint16_t off = 18;
    memcpy(&req[off], pMaskMsg->Data, mlen);
    off = (uint16_t)(off + mlen);
    memcpy(&req[off], pPatternMsg->Data, plen);
    off = (uint16_t)(off + plen);
    if (fclen) {
        memcpy(&req[off], pFlowControlMsg->Data, fclen);
        off = (uint16_t)(off + fclen);
    }
    long rc = dev_request(PT_CMD_START_FILTER, req, off, resp, &resp_len,
                          sizeof(resp));
    if (rc < 0) {
        return xfer_err("PassThruStartMsgFilter");
    }
    if (rc != STATUS_NOERROR) {
        return set_err(rc, "PassThruStartMsgFilter: device returned %ld", rc);
    }
    if (resp_len != 4) return set_err(ERR_FAILED, "PassThruStartMsgFilter: malformed reply");
    *pFilterID = rd_u32(resp);
    dev_log("PassThruStartMsgFilter -> ch=%lu id=%lu", ChannelID, *pFilterID);
    return STATUS_NOERROR;
}

static long api_PassThruStopMsgFilter(unsigned long ChannelID,
                                  unsigned long FilterID)
{
    dev_log("PassThruStopMsgFilter ch=%lu id=%lu", ChannelID, FilterID);
    uint8_t req[8];
    wr_u32(&req[0], ChannelID);
    wr_u32(&req[4], FilterID);
    long rc = dev_request(PT_CMD_STOP_FILTER, req, sizeof(req), NULL, NULL, 0);
    if (rc < 0) {
        return xfer_err("PassThruStopMsgFilter");
    }
    return rc == STATUS_NOERROR
           ? STATUS_NOERROR
           : set_err(rc, "PassThruStopMsgFilter: device returned %ld", rc);
}

/* ---- misc ---- */

static long api_PassThruSetProgrammingVoltage(unsigned long DeviceID,
                                          unsigned long PinNumber,
                                          unsigned long Voltage)
{
    dev_log("PassThruSetProgrammingVoltage pin=%lu volt=0x%08lx", PinNumber,
            Voltage);
    if (!g_open || DeviceID != DEVICE_ID) {
        return set_err(ERR_INVALID_DEVICE_ID,
                       "PassThruSetProgrammingVoltage: bad device id");
    }
    /* Forward to the device backend (do_set_prog_voltage), which enforces the
     * voltage_policy honesty modes and knows this unit's real capability.
     *
     * HARDWARE NOTE (Ford-flavour Nano): the only pin that sources anything is
     * J1962 pin 13/FEPS, and it is a hard ~18 V on/off, not a variable rail;
     * pins 6/9/11/12/14/15 source nothing.  Depending on voltage_policy the
     * backend may return ERR_PIN_INVALID for a pin it cannot drive rather than
     * silently succeeding.  Other-OEM variants of this device differ -- do not
     * assume pin 13 / 18 V elsewhere.  See device_vcx.c's capability block. */
    uint8_t req[8];
    wr_u32(&req[0], PinNumber);
    wr_u32(&req[4], Voltage);
    long rc = dev_request(PT_CMD_SET_PROG_VOLTAGE, req, sizeof(req), NULL, NULL, 0);
    if (rc < 0) {
        return xfer_err("PassThruSetProgrammingVoltage");
    }
    return rc == STATUS_NOERROR
           ? STATUS_NOERROR
           : set_err(rc, "PassThruSetProgrammingVoltage: device returned %ld", rc);
}

static long api_PassThruReadVersion(unsigned long DeviceID,
                                char *pFirmwareVersion, char *pDllVersion,
                                char *pApiVersion)
{
    if (!pFirmwareVersion || !pDllVersion || !pApiVersion) {
        return set_err(ERR_NULL_PARAMETER,
                       "PassThruReadVersion: NULL parameter");
    }
    if (!g_open || DeviceID != DEVICE_ID) {
        return set_err(ERR_INVALID_DEVICE_ID,
                       "PassThruReadVersion: bad device id");
    }
    strcpy(pFirmwareVersion, g_fw_version);
    strcpy(pDllVersion, DLL_VERSION_FULL);
    strcpy(pApiVersion, API_VERSION_STR);
    dev_log("PassThruReadVersion -> firmware=%s dll=%s api=%s", g_fw_version,
            DLL_VERSION_FULL, API_VERSION_STR);
    return STATUS_NOERROR;
}

long WINAPI PassThruGetLastError(char *pErrorDescription)
{
    if (!pErrorDescription) {
        return ERR_NULL_PARAMETER;
    }
    AcquireSRWLockShared(&g_error_lock);
    strcpy(pErrorDescription, g_last_error);
    ReleaseSRWLockShared(&g_error_lock);
    return STATUS_NOERROR;
}

static long api_PassThruIoctl(unsigned long ChannelID, unsigned long IoctlID,
                          void *pInput, void *pOutput)
{
    dev_log("PassThruIoctl ch=%lu id=%lu", ChannelID, IoctlID);
    if (!g_open) return set_err(ERR_INVALID_DEVICE_ID, "PassThruIoctl: device not open");
    if (IoctlID == J2534_IOCTL_READ_VBATT || IoctlID == J2534_IOCTL_READ_PROG_VOLTAGE) {
        if (ChannelID != DEVICE_ID)
            return set_err(ERR_INVALID_DEVICE_ID, "PassThruIoctl: bad device id");
    } else if (!dev_channel_find(ChannelID)) {
        return set_err(ERR_INVALID_CHANNEL_ID, "PassThruIoctl: bad channel");
    }
    /* SET_CONFIG uses bytes 12..(12 + 8*n - 1), so 64 entries need 524 bytes. */
    uint8_t req[12 + 8 * 64], resp[8 * 64];
    uint16_t resp_len = 0;
    wr_u32(&req[0], ChannelID);
    wr_u32(&req[4], IoctlID);
    uint16_t in_len = 8;

    switch (IoctlID) {
    case 0x8004: /* START_REPEAT_MESSAGE */
    case 0x8005: /* QUERY_REPEAT_MESSAGE */
    case 0x8006: { /* STOP_REPEAT_MESSAGE */
        log_repeat_ioctl(ChannelID, IoctlID, pInput, pOutput);
#ifdef VCX_MINIMAL_PROFILE
        /* Stock status: the vendor DLL does not implement Honda's repeat
         * IOCTLs, and HDS's capability probe reads 15 as "unsupported". */
        return set_err(ERR_INVALID_IOCTL_ID, "repeat IOCTL 0x%04lX: not in the "
                       "minimal profile", IoctlID);
#else
        long rc;
        uint32_t id = 0, active = 0;
        SIZE_T copied = 0;
        dev_channel_t *ch = dev_channel_find(ChannelID);
        if (!ch) {
            rc = ERR_INVALID_CHANNEL_ID;
        } else if (ch->protocol != J2534_ISO9141) {
            /* HDS's probe interprets any other error as feature support. */
            rc = ERR_INVALID_IOCTL_ID;
        } else if (IoctlID == 0x8004) {
            repeat_log_input input;
            if (!pInput || !pOutput || !ReadProcessMemory(GetCurrentProcess(), pInput,
                    &input, sizeof(input), &copied) || copied != sizeof(input)) {
                rc = ERR_NULL_PARAMETER;
            } else if (input.tx.ProtocolID != ch->protocol || input.mask.ProtocolID != ch->protocol ||
                       input.pattern.ProtocolID != ch->protocol) {
                dev_log("repeat START protocol mismatch: tx=%lu mask=%lu pattern=%lu channel=%lu",
                        input.tx.ProtocolID, input.mask.ProtocolID, input.pattern.ProtocolID,
                        (unsigned long)ch->protocol);
                rc = ERR_MSG_PROTOCOL_ID;
            } else if (!input.tx.DataSize || input.tx.DataSize > sizeof(input.tx.Data) ||
                       !input.mask.DataSize || input.mask.DataSize > sizeof(input.mask.Data) ||
                       input.mask.DataSize != input.pattern.DataSize) {
                rc = ERR_INVALID_MSG;
            } else {
                rc = dev_repeat_start(ChannelID, input.interval, input.condition, input.tx.TxFlags,
                    input.tx.Data, (uint16_t)input.tx.DataSize, input.mask.Data, input.pattern.Data,
                    (uint16_t)input.mask.DataSize, &id);
                if (rc == STATUS_NOERROR) {
                    if (!WriteProcessMemory(GetCurrentProcess(), pOutput, &id, sizeof(id), &copied) || copied != sizeof(id)) {
                        dev_repeat_stop(ChannelID, id); rc = ERR_NULL_PARAMETER;
                    } else { dev_repeat_activate(ChannelID); active = 1; }
                }
            }
        } else if (!pInput || !ReadProcessMemory(GetCurrentProcess(), pInput, &id, sizeof(id), &copied) || copied != sizeof(id)) {
            rc = ERR_NULL_PARAMETER;
        } else if (IoctlID == 0x8006) {
            rc = dev_repeat_stop(ChannelID, id);
        } else if (!pOutput) {
            rc = ERR_NULL_PARAMETER;
        } else {
            rc = dev_repeat_query(ChannelID, id, &active);
            if (rc == STATUS_NOERROR && (!WriteProcessMemory(GetCurrentProcess(), pOutput,
                    &active, sizeof(active), &copied) || copied != sizeof(active))) rc = ERR_NULL_PARAMETER;
        }
        dev_log("repeat result ch=%lu ioctl=0x%04lX id=%lu active=%lu rc=%ld",
                ChannelID, IoctlID, (unsigned long)id, (unsigned long)active, rc);
        return rc == STATUS_NOERROR ? rc : set_err(rc, "PassThruIoctl: repeat ioctl 0x%04lX failed (%ld)", IoctlID, rc);
#endif
    }
    case J2534_IOCTL_FIVE_BAUD_INIT: {
        SBYTE_ARRAY *input = pInput, *output = pOutput;
        dev_channel_t *ch = dev_channel_find(ChannelID);
        if (!ch) return set_err(ERR_INVALID_CHANNEL_ID, "FIVE_BAUD_INIT: bad channel");
        if (!input || !output || !input->BytePtr || !output->BytePtr)
            return set_err(ERR_NULL_PARAMETER, "FIVE_BAUD_INIT: NULL parameter");
        if (input->NumOfBytes != 1)
            return set_err(ERR_INVALID_IOCTL_VALUE, "FIVE_BAUD_INIT: expected one target byte");
        if (output->NumOfBytes < 2)
            return set_err(ERR_INVALID_IOCTL_VALUE, "FIVE_BAUD_INIT: output needs two bytes");
        uint32_t proto = ch->protocol;
        /* VCXPT32 0x10009510 also accepts UART_ECHO_BYTE_PS (KW1281 engine
         * 0x9204, whose write path 0x080348F6 honours flag 0x08); its
         * FAST_INIT list at 0x10009392 does not. */
        if (!(proto == J2534_ISO9141 || proto == J2534_ISO14230 ||
              proto == J2534_2_ISO9141_PS || proto == J2534_2_ISO14230_PS ||
              proto == J2534_2_HONDA_DIAGH_PS || proto == J2534_2_UART_ECHO_BYTE_PS))
            return set_err(ERR_NOT_SUPPORTED, "FIVE_BAUD_INIT: protocol %lu", proto);
        uint64_t generation;
        long clear_rc = dev_channel_clear_rx(ChannelID, &generation);
        if (clear_rc != STATUS_NOERROR) return set_err(clear_rc, "init: channel closed");
        output->NumOfBytes = 0;
        uint32_t worst_ms;
        long prep = dev_five_baud_begin(ChannelID, &worst_ms);
        if (prep < 0) return xfer_err("FIVE_BAUD_INIT");
        if (prep != STATUS_NOERROR) return set_err(prep, "FIVE_BAUD_INIT: idle setup failed (%ld)", prep);
        long rc = send_uart_init(ChannelID, 0x08, input->BytePtr, 1);
        if (rc != STATUS_NOERROR) {
            dev_five_baud_end(ChannelID, false);
            if (rc < 0) return xfer_err("FIVE_BAUD_INIT");
            return set_err(rc, "FIVE_BAUD_INIT: device returned %ld", rc);
        }
        DWORD timeout = five_baud_timeout(worst_ms);
        dev_log("  FIVE_BAUD_INIT: waiting up to %lu ms (firmware worst case %lu ms)",
                (unsigned long)timeout, (unsigned long)worst_ms);
        long init_status;
        rx_msg_t *m = wait_uart_init_reply(ChannelID, generation, timeout, true, 0, 0,
                                           &init_status);
        if (!m) {
            dev_five_baud_end(ChannelID, true);
            return set_err(init_status, "FIVE_BAUD_INIT: no valid sync/key-byte record");
        }
        unsigned baud = (unsigned)(m->data[3] << 8 | m->data[4]);
#ifndef VCX_MINIMAL_PROFILE
        if (baud < PT_FIVE_BAUD_MIN_BAUD || baud > PT_FIVE_BAUD_MAX_BAUD) {
            /* The firmware never checks the sync byte; a bogus measurement
             * means the key bytes cannot be trusted either. */
            free(m);
            dev_five_baud_end(ChannelID, true);
            return set_err(ERR_FAILED, "FIVE_BAUD_INIT: implausible sync baud %u", baud);
        }
#endif
        dev_five_baud_end(ChannelID, false);
        dev_log("  FIVE_BAUD_INIT ok: key bytes %02X %02X, measured sync baud %u",
                m->data[1], m->data[2], baud);
        output->BytePtr[0] = m->data[1];
        output->BytePtr[1] = m->data[2];
        output->NumOfBytes = 2;
        free(m);
        return STATUS_NOERROR;
    }
    case J2534_IOCTL_FAST_INIT: {
        PASSTHRU_MSG *input = pInput, *output = pOutput;
        dev_channel_t *ch = dev_channel_find(ChannelID);
        if (!ch) return set_err(ERR_INVALID_CHANNEL_ID, "FAST_INIT: bad channel");
        if (input && input->ProtocolID != ch->protocol)
            return set_err(ERR_INVALID_MSG, "FAST_INIT: request protocol %lu != "
                           "channel %lu", input->ProtocolID, ch->protocol);
        long fiv = pt_validate_msg(ch->protocol, PT_OP_FAST_INIT, input ? input->DataSize : 0,
                                   input ? input->TxFlags : 0, dev_strict_validation());
        if (fiv != STATUS_NOERROR)
            return set_err(fiv, "FAST_INIT: invalid request message (size=%lu)",
                           input->DataSize);
        uint32_t proto = ch->protocol;
        if (!(proto == J2534_ISO9141 || proto == J2534_ISO14230 ||
              proto == J2534_2_ISO9141_PS || proto == J2534_2_ISO14230_PS ||
              proto == J2534_2_HONDA_DIAGH_PS))
            return set_err(ERR_NOT_SUPPORTED, "FAST_INIT: protocol %lu", proto);
        uint16_t pdu_len = input ? (uint16_t)input->DataSize : 0;
        uint32_t wait_ms, min_reply_ms;
        long win = dev_fast_init_window(ChannelID, pdu_len, &wait_ms, &min_reply_ms);
        if (win != STATUS_NOERROR) return set_err(win, "FAST_INIT: bad channel");
#ifdef VCX_INIT_TEST
        wait_ms = PT_INIT_TIMEOUT_MS;  /* the harnesses keep their short fixed wait */
#endif
        uint64_t generation;
        long clear_rc = dev_channel_clear_rx(ChannelID, &generation);
        if (clear_rc != STATUS_NOERROR) return set_err(clear_rc, "init: channel closed");
        uint32_t sent_at_us = dev_host_us();
        long rc = send_uart_init(ChannelID, 0x04, input ? input->Data : NULL, pdu_len);
        if (rc < 0) return xfer_err("FAST_INIT");
        if (rc != STATUS_NOERROR) return set_err(rc, "FAST_INIT: device returned %ld", rc);
        if (!output) return STATUS_NOERROR;
        dev_log("  FAST_INIT: waiting up to %lu ms (no reply possible before %lu ms)",
                (unsigned long)wait_ms, (unsigned long)min_reply_ms);
        long init_status;
        rx_msg_t *m = wait_uart_init_reply(ChannelID, generation, wait_ms, false, sent_at_us,
                                           min_reply_ms, &init_status);
        if (!m) return set_err(init_status, "FAST_INIT: no valid response in %lu ms",
                               (unsigned long)wait_ms);
        memset(output, 0, sizeof(*output));
        output->ProtocolID = proto;
        output->RxStatus = m->rx_status;
        output->Timestamp = m->timestamp;
        output->DataSize = m->len;
        output->ExtraDataIndex = m->len;   /* no extra bytes: index == DataSize per J2534 */
        memcpy(output->Data, m->data, m->len);
        free(m);
        return STATUS_NOERROR;
    }
    case J2534_IOCTL_GET_CONFIG: {
        SCONFIG_LIST *list = pInput;
        if (!list || !list->ConfigPtr) {
            return set_err(ERR_NULL_PARAMETER, "GET_CONFIG: NULL list");
        }
        if (list->NumOfParams > 64) {
            return set_err(ERR_INVALID_IOCTL_VALUE, "GET_CONFIG: too many");
        }
        wr_u32(&req[8], list->NumOfParams);
        for (unsigned long i = 0; i < list->NumOfParams; i++) {
            wr_u32(&req[12 + i * 4], list->ConfigPtr[i].Parameter);
            dev_log("  GET_CONFIG param=0x%02lX", list->ConfigPtr[i].Parameter);
        }
        in_len = (uint16_t)(12 + list->NumOfParams * 4);
        long rc = dev_request(PT_CMD_IOCTL, req, in_len, resp, &resp_len,
                              sizeof(resp));
        if (rc < 0) {
            return xfer_err("PassThruIoctl");
        }
        if (rc != STATUS_NOERROR) {
            return set_err(rc, "GET_CONFIG: device returned %ld", rc);
        }
        if (resp_len != list->NumOfParams * 8)
            return set_err(ERR_FAILED, "GET_CONFIG: malformed reply");
        for (unsigned long i = 0; i < list->NumOfParams &&
                                  (i + 1) * 8 <= resp_len; i++) {
            list->ConfigPtr[i].Value = rd_u32(&resp[i * 8 + 4]);
            dev_log("  GET_CONFIG param=0x%02lX -> %lu",
                    list->ConfigPtr[i].Parameter, list->ConfigPtr[i].Value);
        }
        return STATUS_NOERROR;
    }
    case J2534_IOCTL_SET_CONFIG: {
        SCONFIG_LIST *list = pInput;
        if (!list || !list->ConfigPtr) {
            return set_err(ERR_NULL_PARAMETER, "SET_CONFIG: NULL list");
        }
        if (list->NumOfParams > 64) {
            return set_err(ERR_INVALID_IOCTL_VALUE, "SET_CONFIG: too many");
        }
        wr_u32(&req[8], list->NumOfParams);
        for (unsigned long i = 0; i < list->NumOfParams; i++) {
            wr_u32(&req[12 + i * 8], list->ConfigPtr[i].Parameter);
            wr_u32(&req[16 + i * 8], list->ConfigPtr[i].Value);
            /* LOOPBACK (0x03) in particular: the transport honours it
             * host-side, so its value decides what WriteMsgs echoes back. */
            dev_log("  SET_CONFIG param=0x%02lX value=%lu",
                    list->ConfigPtr[i].Parameter, list->ConfigPtr[i].Value);
        }
        in_len = (uint16_t)(12 + list->NumOfParams * 8);
        break;
    }
    case J2534_IOCTL_READ_VBATT: {
        if (!pOutput) {
            return set_err(ERR_NULL_PARAMETER, "READ_VBATT: NULL output");
        }
        long rc = dev_request(PT_CMD_IOCTL, req, in_len, resp, &resp_len,
                              sizeof(resp));
        if (rc < 0) {
            return xfer_err("PassThruIoctl");
        }
        if (rc != STATUS_NOERROR) {
            return set_err(rc, "READ_VBATT: device returned %ld", rc);
        }
        if (resp_len != 4) return set_err(ERR_FAILED, "READ_VBATT: malformed reply");
        *(unsigned long *)pOutput = rd_u32(resp);
        dev_log("  READ_VBATT -> %lu mV", *(unsigned long *)pOutput);
        return STATUS_NOERROR;
    }
    case J2534_IOCTL_READ_PROG_VOLTAGE: {
        if (!pOutput) {
            return set_err(ERR_NULL_PARAMETER, "READ_PROG_VOLTAGE: NULL output");
        }
        long rc = dev_request(PT_CMD_IOCTL, req, in_len, resp, &resp_len,
                              sizeof(resp));
        if (rc < 0) {
            return xfer_err("PassThruIoctl");
        }
        if (rc != STATUS_NOERROR) {
            return set_err(rc, "READ_PROG_VOLTAGE: device returned %ld", rc);
        }
        if (resp_len != 4) return set_err(ERR_FAILED, "READ_PROG_VOLTAGE: malformed reply");
        *(unsigned long *)pOutput = rd_u32(resp);
        dev_log("  READ_PROG_VOLTAGE -> %lu mV", *(unsigned long *)pOutput);
        return STATUS_NOERROR;
    }
    case J2534_IOCTL_CLEAR_RX_BUFFER:
        break; /* backend clears queue, overflow and event under one lock */
    case J2534_IOCTL_CLEAR_TX_BUFFER:
    case J2534_IOCTL_CLEAR_PERIODIC_MSGS:
    case J2534_IOCTL_CLEAR_MSG_FILTERS:
        break; /* no translation needed */
    case J2534_IOCTL_ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
    case J2534_IOCTL_DELETE_FROM_FUNCT_MSG_LOOKUP_TBL: {
        /* SAE J2534-1: both IOCTLs take an SBYTE_ARRAY whose BytePtr[n] are
         * functional addresses, not a PASSTHRU_MSG. */
        SBYTE_ARRAY *input = pInput;
        dev_channel_t *ch = dev_channel_find(ChannelID);
        if (!ch) return set_err(ERR_INVALID_CHANNEL_ID, "FUNCT_MSG_LOOKUP_TABLE: bad channel");
        if (!input || (input->NumOfBytes && !input->BytePtr))
            return set_err(ERR_NULL_PARAMETER, "FUNCT_MSG_LOOKUP_TABLE: NULL parameter");
        if (!(ch->protocol == J2534_J1850PWM || ch->protocol == J2534_J1850VPW ||
              ch->protocol == J2534_2_J1850PWM_PS || ch->protocol == J2534_2_J1850VPW_PS))
            return set_err(ERR_NOT_SUPPORTED, "FUNCT_MSG_LOOKUP_TABLE: protocol %lu",
                           ch->protocol);
        if (input->NumOfBytes == 0) return STATUS_NOERROR;   /* nothing to add/delete */
        if (input->NumOfBytes > 1)
            return set_err(ERR_NOT_SUPPORTED,
                           "FUNCT_MSG_LOOKUP_TABLE: device has one functional-address slot");
        wr_u32(&req[8], input->BytePtr[0]);
        in_len = 12;
        break;
    }
    case J2534_IOCTL_CLEAR_FUNCT_MSG_LOOKUP_TABLE: {
        dev_channel_t *ch = dev_channel_find(ChannelID);
        if (!pt_proto_is_j1850(ch->protocol))
            return set_err(ERR_NOT_SUPPORTED, "CLEAR_FUNCT_MSG_LOOKUP_TABLE: protocol %lu", ch->protocol);
        break;
    }
    default:
        /* A documented ioctl this DLL has no case for is NOT_SUPPORTED -- the
         * request was well-formed and this device cannot honour it, which is
         * what a capability probe walking the J2534-2 ioctls needs to hear.
         * Only an id outside the documented range is an invalid argument.  Same
         * distinction pin_is_prog_voltage_arg() draws for programming-voltage
         * pins; answering INVALID_IOCTL_ID for both was the inverse of it. */
        if (j2534_ioctl_id_defined(IoctlID))
            return set_err(ERR_NOT_SUPPORTED, "PassThruIoctl: ioctl %lu is "
                           "defined but not implemented on this device", IoctlID);
        return set_err(ERR_INVALID_IOCTL_ID, "PassThruIoctl: unknown ioctl %lu",
                       IoctlID);
    }

    long rc = dev_request(PT_CMD_IOCTL, req, in_len, resp, &resp_len,
                          sizeof(resp));
    if (rc < 0) {
        return xfer_err("PassThruIoctl");
    }
    return rc == STATUS_NOERROR
           ? STATUS_NOERROR
           : set_err(rc, "PassThruIoctl: device returned %ld", rc);
}

/* Validation, device transactions and channel registration are one mutation.
 * The backend uses this same recursive lock for physical teardown/handoff.
 * ReadMsgs deliberately stays outside it so a blocking read does not prevent
 * transmission or disconnect; its queue operations check a captured generation.
 * Error text has its own lock, also usable without an open device. */
#define GUARDED_API(name, params, args) \
    long WINAPI name params { \
        dev_api_lock(); \
        long rc = api_##name args; \
        dev_api_unlock(); \
        return rc; \
    }

GUARDED_API(PassThruOpen, (void *name, unsigned long *id), (name, id))
GUARDED_API(PassThruClose, (unsigned long id), (id))
GUARDED_API(PassThruConnect,
    (unsigned long id, unsigned long proto, unsigned long flags, unsigned long baud, unsigned long *channel),
    (id, proto, flags, baud, channel))
GUARDED_API(PassThruDisconnect, (unsigned long channel), (channel))
GUARDED_API(PassThruWriteMsgs,
    (unsigned long channel, PASSTHRU_MSG *msg, unsigned long *count, unsigned long timeout),
    (channel, msg, count, timeout))
GUARDED_API(PassThruStartPeriodicMsg,
    (unsigned long channel, PASSTHRU_MSG *msg, unsigned long *id, unsigned long interval),
    (channel, msg, id, interval))
GUARDED_API(PassThruStopPeriodicMsg, (unsigned long channel, unsigned long id), (channel, id))
GUARDED_API(PassThruStartMsgFilter,
    (unsigned long channel, unsigned long type, PASSTHRU_MSG *mask, PASSTHRU_MSG *pattern,
     PASSTHRU_MSG *flow, unsigned long *id),
    (channel, type, mask, pattern, flow, id))
GUARDED_API(PassThruStopMsgFilter, (unsigned long channel, unsigned long id), (channel, id))
GUARDED_API(PassThruSetProgrammingVoltage,
    (unsigned long device, unsigned long pin, unsigned long voltage), (device, pin, voltage))
GUARDED_API(PassThruReadVersion,
    (unsigned long device, char *fw, char *dll, char *api), (device, fw, dll, api))
GUARDED_API(PassThruIoctl,
    (unsigned long channel, unsigned long id, void *input, void *output), (channel, id, input, output))
#undef GUARDED_API

/* ---- DLL entry ---- */

BOOL WINAPI DllMain(HINSTANCE hinst, DWORD reason, LPVOID reserved)
{
    if (reason == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(hinst);
        dev_init(hinst);
    } else if (reason == DLL_PROCESS_DETACH && reserved == NULL) {
        /* A backend with workers pins the module before starting them, so
         * explicit unload reaches here only before any worker has started.
         * Process termination requires no cleanup of killed threads. */
        dev_shutdown();
    }
    return TRUE;
}
