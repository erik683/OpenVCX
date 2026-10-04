# Developing OpenVCX

Keep changes focused. For a bug report, include the app/version, Nano variant,
firmware, DLL build/hash, symptoms and a short log excerpt. Remove VINs and other
private data. For hardware results, say what vehicle and operation actually worked.

## Build

Install Visual Studio Build Tools with **Desktop development with C++** and a
Windows SDK, or use MinGW-w64 GCC targeting each requested architecture.
The launcher finds a compiler or asks for its path:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\build_DLL.ps1 -Archs x86,x64
```

`build.cmd` runs the same launcher. To build only one architecture, use `-Archs x86`
or `-Archs x64`, then `install.cmd -Arch x86` or `install.cmd -Arch x64`.
For a DLL-only development build, use `build_DLL_core.ps1 -DllOnly -Archs x86`.

## Tests

Normal builds run seven offline harnesses covering argument validation, wire
framing, configuration, initialization, API behavior, lifecycle and DLL unloading,
plus a DLL load/14-export check. These use mocks or no-device paths.
`dll_smoke` without `exports-only` and `handoff_test` need hardware and are not
run automatically.

CI builds/tests both architectures with MSVC. The initial import also passed
both architectures with MinGW GCC 16.2.0 and MSVC 19.51; that was an offline check.

## Code map

| File | Responsibility |
|---|---|
| [api.c](dll/api.c) | J2534 entry points and channel/message handling |
| [device_vcx.c](dll/device_vcx.c) | Nano commands, routing, configuration and repeat scheduling |
| [vcx_transport.c](dll/vcx_transport.c) | Wire framing/parser |
| [pt_validate.c](dll/pt_validate.c) | API argument validation |
| [ini_config.h](dll/ini_config.h) | Configuration keys |
| [j2534.h](dll/j2534.h), [j2534_defs.h](dll/j2534_defs.h) | API definitions and constants |

## Known implementation gaps

User-visible Honda and hardware limits are in the [README](README.md).
The main additional constraints for client authors are:

- **Timeout recovery:** a late control reply can be mistaken for a later request
  with the same opcode/channel. Reconnect after transport failures.
- **Shared resources:** UART-family protocols share one hardware instance;
  exclusive ownership is not enforced. Channels on one CAN controller share its bitrate.
- **Rejected pin changes:** the old/default route remains active. An application
  that ignores the error may still transmit on pin 7.
- **Configuration:** `BS_TX`, `STMIN_TX`, `ISO15765_WFT_MAX` and `W0` are
  accepted/cached without implementing their full requested behavior.
- **Timing/buffers:** TX indications mean serial submission. Timestamps use the
  host queue clock. `CLEAR_RX_BUFFER` clears the host queue; `CLEAR_TX_BUFFER`
  is a compatibility no-op.
- **Honda:** DIAG-H messages on a base ISO9141 periodic channel are rejected.
  Overlapping repeat/keep-alive behavior needs more vehicle testing.
- **Optional APIs:** many J2534-2 IOCTLs are unsupported. TP1.6/TP2.0 firmware
  filter setters are stubs. Full J2534 conformance is not claimed.

### Experimental protocol IDs

These need a client that requests them; a successful Connect is not a complete
vehicle session.

| Protocol | IDs (hex) |
|---|---|
| Honda DIAG-H alias to ISO9141 | `800B` |
| UART Echo Byte / KW1281 | `800A` |
| Private J1939 / J1708 | `800C` / `800D` |
| LIN | `8020` |
| TP2.0 | `800E`, `8021` |
| TP1.6 / KW82 | `8022` / `8023` |

Private ISO-TP configuration is defined in [j2534_defs.h](dll/j2534_defs.h):
`18001` address mode, `18007` initial TX block size, `18008` raw pacing,
`18009` timeout multiplier, `1800A` timeout in microseconds. These are firmware
controls, not substitutes with identical semantics for standard J2534 settings.

## Releases

Run `powershell -NoProfile -ExecutionPolicy Bypass -File .\package_release.ps1 -Version 0.1.0`
with the intended release version. It builds a standalone source snapshot, runs
both architectures' offline tests/export checks, and writes `dist/OpenVCX-<version>.zip`
plus its SHA-256. Existing ZIPs are not overwritten.

The ZIP contains `bin/x86/OpenVCX32.dll`, `bin/x64/OpenVCX64.dll`, the installer,
docs, licenses, complete corresponding source and `SHA256SUMS.txt`.
`release-info.txt` records compiler versions and source fingerprints. Snapshot
builds have a `nogit` build ID; ordinary checkout builds also include the Git revision.
Build logs and test executables stay under `build/release-*`.
Check real install/update/uninstall behavior on Windows before publishing.

Keep binaries, compiler bundles, firmware, vendor files and private captures out
of Git. Maintain the public DLL here; transfer research fixes deliberately.
Capability percentages in the README are maintainer estimates, not test statistics.

## License and origins

Copyright (c) 2026 Erik Fuller and OpenVCX contributors.
Source, scripts, tests and original documentation are LGPL-3.0-only unless a file
states otherwise. Preserve notices and the [LGPL](LICENSE)/[GPL](COPYING) texts.

OpenVCX originated in the VCX Nano research project. Its API framework came from
work commissioned and owned by Erik Fuller on a local J2534-prototype /
esp-passthru project; the serial backend was rewritten for the Nano.
Vendor DLLs, firmware, drivers and diagnostic software are not included.

Vehicle summaries come from the maintainer's Mustang/FORScan/FEPS confirmation
and recorded Civic/HDS sessions. The private research logs are not distributed here.
