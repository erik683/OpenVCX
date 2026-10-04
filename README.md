# OpenVCX J2534 DLL/API for the VCX Nano automotive diagnostic tool series

**WARNING: This is dealer-level full-access J2534 software that is intended to function with automotive diagnostics software. It can reprogram modules, operate vehicle systems, and apply programming voltage. Mistakes can brick an ECU, damage electronics, or cause injury. Know what the procedure does before running it.**

OpenVCX is an open-source Windows J2534 DLL for VCX Nano passthrough adapters.
It replaces the original vendor PassThru DLL so compatible applications can talk directly
to the Nano, without needing to install VX Manager or the vendor DLLs.

Communication flows: **Diagnostic application** -> **OpenVCX** -> **VCX Nano** -> **Vehicle**

You still need a USB serial driver for the Nano's CH343 USB chip and J2534-compatible diagnostic software.
The driver is available directly from the chip maker, WCH
([CH343SER for Windows](https://www.wch-ic.com/downloads/CH343SER_EXE.html)); VX Manager is not required.
OpenVCX uses the existing device firmware and entitlement; it does not flash the
adapter or provide OEM software licenses.

## What's working now

Percentages are rough completion estimates for the uses listed, not measured
success rates. Anything less than 100% means not everything has been tested IRL.

| Protocol                                   | Curr. Status |    %   | What works                                                                |
|--------------------------------------------|--------------|--------|---------------------------------------------------------------------------|
| Classic CAN: HS-CAN and MS-CAN             |  Works       |  ~95%  | Send/receive, filters and periodic messages on pins 6/14 and 3/11.        |
| ISO 15765-2 (ISO-TP over CAN)              |  Works       |  ~95%  | Vehicle identification, DTCs and live data in FORScan and HDS.            |
| ISO 9141 / Honda K-line                    |  Works / WIP |  ~85%  | Module scans, data streaming and DTCs. Minor comms issues.                |
| ISO 14230 (KWP2000)                        |  Works / WIP |  ~85%  | Successful module discovery and diagnostic exchanges on the K-line.       |
| Ford FEPS programming voltage              |  Works       |  100%  | Fixed ~18 V on pin 13; FEPS programming required. As advertised, finally. |
| Vehicle battery voltage                    |  Works       |  100%  | Reads supply voltage from pin 16.                                         |
| J1850, J1708, LIN, TP1.6/2.0, KW1281, KW82 |  Untested    |  ???   | Firmware code paths exist; not yet tested but implemented in DLL.         |
| Standard J2534-2 J1939                     |  Not working |  ???   | Firmware code paths exist; the standard API/transport is unfinished.      |
| J1939 through raw CAN                      |  Possible    |  ???   | A custom application could technically implement J1939 over 29-bit CAN.   |
| CAN FD                                     |  Not working |  N/A   | No CAN FD controller on the PCB.                                          |

### Ford and FORScan

**By and large, FORScan works, including extended features.** The maintainer
confirms Ford diagnostics and FEPS programming on a **2006 Ford Mustang**.
Other models have not been confirmed yet.

FDRS and other applications that accept a J2534 provider should be candidates,
but **have not been tested**. Ford IDS uses a different interface and does not
work through this DLL alone.

### Ford FEPS: a fix over the original DLL

FEPS programming typically doesn't work with the original DLL EVEN THOUGH the
Ford version of Nano has the hardware and firmware code for it. With OpenVCX
it does, at least on a 2006 Mustang PCM which means it will likely work across
the 2005-2009 Mustang generation at the very least, possibly for all FEPS generation
vehicles. The Ford/Mazda Nano has the PCB connections and components necessary
to supply **18 V on OBD pin 13**.

OpenVCX translates the application's programming-voltage request into the Nano's
existing rail on/off command. It restricts that command to pin 13 and switches
the fixed 18 V supply instead of treating it as an adjustable voltage source.
No PCB modification or replacement firmware is needed. Other Ford models and
other Nano variants remain untested.

### Honda HDS

**Honda works. HDS compatibility is mostly complete, with some bugs.**
Testing used a **2011 Honda Civic** and **HDS PC 3.102.051**. A full five-system
scan covered PGM-FI, SRS, ABS, TPMS and Body Electrical. Live data, K-line/KWP
communication, and ABS/TPMS fault clearing have worked.

The biggest remaining issues are intermittent K-line wake-up failures,
communication stopping after a clear, and unsupported alternate-pin requests.
A fix for the post-clear stall is included but still needs a vehicle retest.
Reconnect if a session stops responding. I suggest avoid using OBD/DLC
extenders or breakouts; the K-line connection can be electrically finicky.

## Hardware and pin limits

Nano models differ. My test unit is a **Ford/Mazda-branded USB Nano**:
NANO-MC-V1.1 board (2024.01), CH343 USB serial, firmware 1.9.4.2.
The following describes that unit. Wi-Fi/Bluetooth is not implemented; the test unit has no wireless hardware.

|        OBD pins       |          Function        |           Status           |
|-----------------------|--------------------------|----------------------------|
|         6 / 14        |           HS-CAN         |           Working          |
|         3 / 11        |           MS-CAN         |           Working          |
|           7           |      K-line / KWP2000    |   Working, slightly buggy  |
|           13          |          Ford FEPS       |           Working          |
|           16          |         VBAT reading     |           Working          |
|         4 / 5         |           Grounds        |           Working          |
|         2 / 10        |     Possible J1850 use   |           Untested         |
|  1 / 8 / 9 / 12 / 15  |     Other manufacturers  |       No supported route   |

Pin assignments are hardwired. Requests for other CAN pairs or alternate K-line
pins effectively lead nowhere. Only pin 13 supplies programming voltage (on my Ford/Mazda unit),
and only pin 16 measures actual live voltage. L-line on pin 15, adjustable programming voltage
and single-wire/fault-tolerant CAN are not supported. Both CAN buses work individually;
sustained simultaneous dual-bus use still needs testing.

## Get started

1. Use **64-bit Windows 10/11** with the CH343 USB serial driver installed.
   Check that the adapter appears under **Device Manager -> Ports (COM & LPT)**.
2. Extract the **OpenVCX release ZIP** to a local folder such as `C:\OpenVCX`.
   It includes both DLLs, matching source and SHA-256 checksums; no compiler is needed.
   **Code -> Download ZIP** contains source only; see [building from source](CONTRIBUTING.md#build).
3. Close diagnostic applications. Double-click **install.cmd** and accept the
   administrator prompt. It installs and registers both architectures.
4. Restart your diagnostic app and select **OpenVCX32** for a 32-bit app or
   **OpenVCX64** for a 64-bit app. The original vendor provider remains available.

For only one architecture, open PowerShell in the extracted folder and run
`.\install.cmd -Arch x86` or `.\install.cmd -Arch x64`.

### Troubleshooting and updates

|              Problem               |                                      Try this                                                  |
|------------------------------------|------------------------------------------------------------------------------------------------|
|  OpenVCX missing or DLL won't load | Install the architecture (x86/x64) matching the application, then restart it.                  |
|      Device won't open comms       | Check the USB driver/cable and COM port; close VX Manager and other diagnostic apps.           |
|  Connects but module doesn't reply | Check ignition, vehicle selection and supported pins. Try again, wait, try again.              |
| HDS stops responding after a clear | Exit the procedure and reconnect. Clear might have succeeded                                   |

To select a preferred COM port and enable logging, substitute your port below:

```powershell
.\install.cmd -Port COM14 -Log
```

The log goes to `logs\openvcx.log` in the extracted folder; keep that folder when logging.
See [configuration](CONFIGURATION.md) for settings.
For an update, extract the new release into a fresh folder and run its installer; it overwrites
the INI file, so save custom settings first.

To remove both architectures:

```powershell
.\install.cmd -Uninstall
# Add -Arch x86 or -Arch x64 to remove only one.
```

## Development and license

See [CONTRIBUTING.md](CONTRIBUTING.md) for tests, API gaps and source
origins. Experimental protocol registration is available with
`install.cmd -ExperimentalProtocols`; it does not add features to your diagnostic app.

Licensed under **LGPL-3.0-only**: [LICENSE](LICENSE) and [COPYING](COPYING).
Commercial use is allowed under those terms. This is an independent project,
not an official VXDIAG product.

Use at your own risk. If an airbag launches into you and your laptop, the
consequences are yours; this software comes without warranty.
