/* SPDX-License-Identifier: LGPL-3.0-only
 * Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
 * See LICENSE and COPYING for terms; provided without warranty.
 */
/* Startup-only configuration. Research keys and their environment aliases are
 * unavailable in normal builds. Invalid values fall back to compiled defaults. */
#include <limits.h>
#include <ctype.h>

typedef struct {
    const char *key, *env;
    char value[MAX_PATH];
    bool invalid;
} ini_option_t;
static ini_option_t s_ini[] = {
    {"port", "VCX_NANO_PORT"}, {"log", "VCX_NANO_LOG"},
    {"log_level", "VCX_NANO_LOG_LEVEL"}, {"hex_max", "VCX_NANO_LOG_HEXMAX"},
    {"rx_log_every", "VCX_NANO_LOG_RX_EVERY"}, {"keep_warm", NULL},
    {"repeat_reply_timeout_ms", NULL},
#ifdef VCX_RESEARCH_CONFIG
    {"license_refresh_ms", "VCX_NANO_LICENSE_REFRESH_MS"},
    {"voltage_policy", "VCX_NANO_VOLTAGE_POLICY"},
    {"strict_validation", "VCX_NANO_STRICT"},
    {"can_bus", NULL}, {"kline_pin", NULL}, {"periodic", NULL},
#endif
};

static bool ini_uint(const char *s, unsigned long min, unsigned long max)
{
    if (!*s) return false;
    for (const char *p = s; *p; ++p) if (*p < '0' || *p > '9') return false;
    errno = 0;
    unsigned long n = strtoul(s, NULL, 10);
    return errno != ERANGE && n >= min && n <= max;
}

static bool ini_valid(const char *key, const char *v)
{
    if (!strcmp(key, "log")) return true;
    if (!strcmp(key, "port")) return !_strnicmp(v, "COM", 3) && ini_uint(v + 3, 1, 65535);
    if (!strcmp(key, "log_level")) return ini_uint(v, 1, 2);
    if (!strcmp(key, "hex_max")) return ini_uint(v, 0, INT_MAX);
    if (!strcmp(key, "rx_log_every")) return ini_uint(v, 1, INT_MAX);
    if (!strcmp(key, "repeat_reply_timeout_ms")) return ini_uint(v, 50, 5000);
    if (!strcmp(key, "keep_warm"))
        return !strcmp(v,"0") || !strcmp(v,"1") || !_stricmp(v,"on") || !_stricmp(v,"off") ||
               !_stricmp(v,"true") || !_stricmp(v,"false");
    if (!strcmp(key, "strict_validation"))
        return !strcmp(v,"0") || !strcmp(v,"1") || !_stricmp(v,"on") || !_stricmp(v,"off") ||
               !_stricmp(v,"true") || !_stricmp(v,"false") || !_stricmp(v,"strict") || !_stricmp(v,"lenient");
    if (!strcmp(key, "license_refresh_ms")) return ini_uint(v, 0, INT_MAX);
    if (!strcmp(key, "can_bus")) return ini_uint(v, 0, 1) || !_stricmp(v,"off");
    if (!strcmp(key, "kline_pin")) return ini_uint(v, 1, 16) || !_stricmp(v,"any");
    if (!strcmp(key, "periodic")) return !_stricmp(v,"auto") || !_stricmp(v,"host");
    if (!strcmp(key, "voltage_policy")) return ini_uint(v,0,3) || !_stricmp(v,"strict") ||
        !_stricmp(v,"honest") || !_stricmp(v,"loose") || !_stricmp(v,"compat") || !_stricmp(v,"stock");
    return false;
}

static void ini_load(HINSTANCE module)
{
    char path[MAX_PATH] = "";
    DWORD n = module ? GetModuleFileNameA(module, path, sizeof(path)) : 0;
    char *slash = n && n < sizeof(path) ? strrchr(path, '\\') : NULL;
    if (slash && (size_t)(slash + 1 - path) + sizeof("vcx_nano.ini") <= sizeof(path))
        strcpy(slash + 1, "vcx_nano.ini");
    else path[0] = 0;
    for (unsigned i = 0; i < sizeof(s_ini)/sizeof(s_ini[0]); ++i) {
        ini_option_t *o = &s_ini[i];
        o->value[0] = 0; o->invalid = false;
        const char *env = o->env ? getenv(o->env) : NULL;
        if (env && *env) {
            if (strlen(env) >= sizeof(o->value)) { o->invalid = true; continue; }
            strcpy(o->value, env);
        } else if (*path) {
            DWORD count = GetPrivateProfileStringA("device", o->key, "", o->value, sizeof(o->value), path);
            if (count >= sizeof(o->value)-1) { o->value[0] = 0; o->invalid = true; continue; }
        }
        /* Windows profile APIs do not strip inline semicolon comments. */
        if (strcmp(o->key,"log")) {
            char *comment = strchr(o->value, ';'); if (comment) *comment = 0;
        }
        size_t len = strlen(o->value);
        while (len && isspace((unsigned char)o->value[len-1])) o->value[--len] = 0;
        char *start = o->value;
        while (isspace((unsigned char)*start)) ++start;
        if (start != o->value) memmove(o->value, start, strlen(start)+1);
        if (*o->value && !ini_valid(o->key, o->value)) { o->value[0] = 0; o->invalid = true; }
    }
}

static bool ini_get(const char *key, char *out, size_t cap)
{
    for (unsigned i = 0; i < sizeof(s_ini)/sizeof(s_ini[0]); ++i) {
        if (!strcmp(key, s_ini[i].key) && *s_ini[i].value) {
            size_t n = strlen(s_ini[i].value);
            if (n >= cap) return false;
            memcpy(out, s_ini[i].value, n+1); return true;
        }
    }
    return false;
}
