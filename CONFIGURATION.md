# Configuration and logging

The installer writes `vcx_nano.ini` beside the DLL in
`C:\Program Files (x86)\OpenVCX` (x86) or `C:\Program Files\OpenVCX` (x64).
Editing it requires administrator rights. Restart the diagnostic app after edits;
reinstalling overwrites the file.

```ini
[device]
port=COM14
log=C:\OpenVCX\logs\openvcx.log
log_level=2
hex_max=256
rx_log_every=256
keep_warm=1
repeat_reply_timeout_ms=300
```

Replace the port and path with yours. The log folder must exist and be writable.
`install.cmd -Log` creates it for you. Remove `log=` to disable file logging.

| Key | Default / meaning |
|---|---|
| `port` | Tries COM5, then COM1..32. A configured port is tried first, with fallback. |
| `log` | Off; set a full file path to enable. |
| `log_level` | 2; use 1 to omit frequent API-entry messages. |
| `hex_max` | 256 bytes per hex dump; 0 includes full frames. |
| `rx_log_every` | Logs the first 64 frames, then every 256th; 1 includes every frame. |
| `keep_warm` | 1 keeps the serial connection for handoff between processes; 0 releases it on close. |
| `repeat_reply_timeout_ms` | 300; range 50..5000. Repeat-message fallback timeout when no reply arrives. |
| `fast_init_timeout_ms` | Omit to derive the FAST_INIT wait from channel timing. Optional fixed wait: 100..30000 ms. |

For a short bug capture, use `hex_max=0` and `rx_log_every=1`.
Logs include build identity and may contain VINs and diagnostic/security data;
redact those before sharing. `TXMSG` means serial submission. J2534 RX timestamps
use host QPC arrival estimates in microseconds, not physical bus measurements.

Environment overrides: `VCX_NANO_PORT`, `VCX_NANO_LOG`, `VCX_NANO_LOG_LEVEL`,
`VCX_NANO_LOG_HEXMAX`, `VCX_NANO_LOG_RX_EVERY`.

## Research builds

`build_DLL_core.ps1 -ResearchConfig` also enables `can_bus`, `kline_pin`, `periodic`,
`license_refresh_ms`, `voltage_policy` and `strict_validation`. These can bypass
normal hardware checks or report success without performing an operation.
They are development options, not routine troubleshooting settings; normal builds
ignore them. See [ini_config.h](dll/ini_config.h).

Research output goes under `build/research/<arch>`. The installer uses a normal
local build in `build/<arch>` when present, otherwise the release DLL in `bin/<arch>`.
