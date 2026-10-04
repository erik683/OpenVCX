# SPDX-License-Identifier: LGPL-3.0-only
# Registers (or removes) the OpenVCX J2534 DLL so J2534 apps like FORScan
# (32-bit) or 64-bit diagnostic tools can find it. Non-destructive: adds its
# own provider key and does NOT touch the installed VXDIAG / FDRS entries.
# Must run elevated (writes HKLM).
#
# Usage (elevated PowerShell):
#   .\install_DLL.ps1 [-Port COM5]              # install both, optionally pin the COM port
#   .\install_DLL.ps1 -Arch x64                 # install the 64-bit build
#   .\install_DLL.ps1 -Log                      # also enable diagnostic frame logging
#   .\install_DLL.ps1 -Uninstall [-Arch x64]    # uninstall the matching arch
#   .\install_DLL.ps1 -Verify [-Arch x64]       # compare installed DLL with the build (no elevation)
#
#   .\install_DLL.ps1 -Plan [-ExperimentalProtocols] # read-only registration plan
# Or double-click install.cmd, which self-elevates and calls this script.
param(
    [ValidateSet("x86","x64","both")]
    [string]$Arch = "both",
    [string]$Port = "",
    [switch]$Log,
    [switch]$Uninstall,
    [switch]$Verify,
    [switch]$ExperimentalProtocols,
    [switch]$Plan             # show registration plan only; no files/registry changed
)

$ErrorActionPreference = "Stop"
if (-not [Environment]::Is64BitOperatingSystem -or -not [Environment]::Is64BitProcess) {
    throw "Use 64-bit PowerShell on 64-bit Windows (x86 diagnostic apps are supported)."
}
if (@($Uninstall, $Verify, $Plan).Where({ $_ }).Count -gt 1) {
    throw "Choose only one of -Uninstall, -Verify or -Plan."
}
if ($Port -and $Port -notmatch '^COM[1-9][0-9]*$') {
    throw "Port must be a COM port name, for example COM5."
}

function Get-SourceDll([string]$targetArch) {
    $bits = if ($targetArch -eq "x86") { "32" } else { "64" }
    $built = Join-Path $PSScriptRoot "build\$targetArch\vcxnano_pt$bits.dll"
    # A local rebuild takes precedence over the prebuilt release.
    if (Test-Path -LiteralPath $built -PathType Leaf) { return $built }
    return (Join-Path $PSScriptRoot "bin\$targetArch\OpenVCX$bits.dll")
}

function Test-SourceDll([string]$targetArch) {
    $path = Get-SourceDll $targetArch
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Missing $targetArch DLL. Extract the entire release ZIP or build this architecture first."
    }
    $bytes = [IO.File]::ReadAllBytes($path)
    if ($bytes.Length -lt 64 -or [BitConverter]::ToUInt16($bytes, 0) -ne 0x5A4D) {
        throw "Invalid DLL: $path"
    }
    $pe = [BitConverter]::ToInt32($bytes, 60)
    $machine = if ($targetArch -eq "x86") { 0x14c } else { 0x8664 }
    if ($pe -lt 64 -or $pe -gt ($bytes.Length - 24) -or
        [BitConverter]::ToUInt32($bytes, $pe) -ne 0x4550 -or
        [BitConverter]::ToUInt16($bytes, $pe + 4) -ne $machine -or
        ([BitConverter]::ToUInt16($bytes, $pe + 22) -band 0x2000) -eq 0) {
        throw "Invalid $targetArch DLL architecture/header: $path"
    }
}

# Check both inputs before installing either architecture.
if (-not $Uninstall -and -not $Plan -and -not $Verify) {
    $targets = if ($Arch -eq "both") { @("x86", "x64") } else { @($Arch) }
    foreach ($target in $targets) { Test-SourceDll $target }
}
if ($Arch -eq "both") {
    $options = @{} + $PSBoundParameters
    foreach ($target in @("x86", "x64")) {
        $options.Arch = $target
        & $PSCommandPath @options
    }
    return
}

# PORTABLE: everything is resolved relative to this script, so OpenVCX can be
# copied anywhere (another machine, a USB stick) and installed from there.
$root = $PSScriptRoot
if ($Arch -eq "x64") {
    # 64-bit apps read the native (non-WOW6432Node) registry view.
    $installDir = "$env:ProgramFiles\OpenVCX"
    $regKey     = "HKLM:\SOFTWARE\PassThruSupport.04.04\OpenVCX"
    $dllName    = "OpenVCX64.dll"
    $providerName = "OpenVCX64"
} else {
    # 32-bit apps (FORScan) read the WOW6432Node view on 64-bit Windows.
    $installDir = "${env:ProgramFiles(x86)}\OpenVCX"
    $regKey     = "HKLM:\SOFTWARE\WOW6432Node\PassThruSupport.04.04\OpenVCX"
    $dllName    = "OpenVCX32.dll"
    $providerName = "OpenVCX32"
}
$srcDll = Get-SourceDll $Arch

# Registry flags describe advertised discovery scope, not certification.
# Keep optional flags explicit (zero) so reinstalling resets an older broad entry.
$experimental = [int][bool]$ExperimentalProtocols
$flags = [ordered]@{
    CAN=1; ISO15765=1; ISO9141=1; ISO14230=1;
    CAN_PS=1; ISO15765_PS=1; ISO9141_PS=1; ISO14230_PS=1;
    J1850PWM=$experimental; J1850VPW=$experimental;
    J1850PWM_PS=$experimental; J1850VPW_PS=$experimental;
    HONDA_DIAGH=$experimental; HONDA_DIAGH_PS=$experimental;
    UART_ECHO_BYTE_PS=$experimental; TP2_0_PS=$experimental;
    # These two names are custom discovery metadata, not standard API promises.
    J1708=$experimental; LIN=$experimental;
    # Mapped aliases/raw engine access do not implement these capabilities.
    J1939=0; SW_CAN_PS=0; FT_CAN_PS=0; SW_ISO15765_PS=0; FT_ISO15765_PS=0
}
if ($Plan) {
    [pscustomobject]@{
        Architecture = $Arch; Provider = $providerName; SourceDLL = $srcDll;
        InstallDirectory = $installDir; RegistryKey = $regKey;
        Experimental = [bool]$ExperimentalProtocols; ProtocolFlags = $flags
    }
    return
}

function Get-Sha256([string]$path) { (Get-FileHash $path -Algorithm SHA256).Hash.ToLower() }

if ($Verify) {
    $dllPath = Join-Path $installDir $dllName
    if (-not (Test-Path $dllPath)) { throw "$dllPath is not installed" }
    $installed = Get-Sha256 $dllPath
    Write-Host "installed: $dllPath"
    Write-Host "  sha256:  $installed"
    if (-not (Test-Path $srcDll)) { throw "$srcDll not found - nothing to compare against" }
    $built = Get-Sha256 $srcDll
    Write-Host "build:     $srcDll"
    Write-Host "  sha256:  $built"
    if ($installed -ne $built) { throw "MISMATCH: the installed DLL is not this build ($Arch)" }
    Write-Host "match"
    return
}

$principal = New-Object Security.Principal.WindowsPrincipal(
    [Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw "Run from an elevated (Administrator) PowerShell."
}

if ($Uninstall) {
    if (Test-Path $regKey)    { Remove-Item $regKey -Recurse -Force -Confirm:$false; Write-Host "removed $regKey" }
    # Remove this arch's DLL, and the vcx_nano.ini this script wrote alongside
    # it. The ini has to go too: install always writes one, so leaving it behind
    # meant the directory was never empty (the cleanup below could never fire)
    # and a stale pinned COM port / voltage_policy survived an uninstall to be
    # picked up by the next install. Each arch has its own $installDir, so this
    # never touches the other arch's settings.
    $dllPath = Join-Path $installDir $dllName
    if (Test-Path $dllPath) { Remove-Item $dllPath -Force; Write-Host "removed $dllPath" }
    $iniPath = Join-Path $installDir "vcx_nano.ini"
    if (Test-Path $iniPath) { Remove-Item $iniPath -Force; Write-Host "removed $iniPath" }
    if ((Test-Path $installDir) -and -not (Get-ChildItem $installDir -Force)) {
        Remove-Item $installDir -Force
        Write-Host "removed empty $installDir"
    }
    return
}

New-Item -ItemType Directory -Force $installDir | Out-Null
Copy-Item $srcDll (Join-Path $installDir $dllName) -Force
$builtHash = Get-Sha256 $srcDll
$installedHash = Get-Sha256 (Join-Path $installDir $dllName)
if ($installedHash -ne $builtHash) {
    throw "installed DLL hash $installedHash does not match build $builtHash - is the old DLL still loaded by a running app?"
}

# vcx_nano.ini next to the DLL, read by dev_connect()/dev_init(): optional COM-port
# preference, diagnostic logging, and connection lifetime.
# NOTE: this file is rewritten on every install - re-add any hand-edited keys after.
$ini = "$installDir\vcx_nano.ini"
$portLine = if ($Port) { "port=$Port" } else { "; port=COM5   ; uncomment to pin, else COM5 then scan" }
$logPath  = "$root\logs\openvcx.log"
$logLine  = if ($Log) { "log=$logPath" } else { "; log=$logPath" }
# Normal configuration: connection lifetime and diagnostic capture only.
$warmLine = "; keep_warm=1   ; default: retain the COM link with cross-process handoff"
$hexLine  = "; hex_max=256   ; 0 = full hex dumps"
$lvlLine  = "; log_level=2   ; 1 = omit high-frequency read/write entry lines"
$rxLine   = "; rx_log_every=1   ; log every Nth received frame"
$repeatLine = "; repeat_reply_timeout_ms=300   ; no-reply repeat fallback, 50-5000 ms"
"[device]`r`n$portLine`r`n$logLine`r`n$warmLine`r`n$hexLine`r`n$lvlLine`r`n$rxLine`r`n$repeatLine`r`n" | Set-Content -Path $ini -Encoding ascii
if ($Port) { Write-Host "pinned port: $Port" }
if ($Log)  { New-Item -ItemType Directory -Force "$root\logs" | Out-Null; Write-Host "logging to: $logPath" }

New-Item -Path $regKey -Force | Out-Null
Set-ItemProperty -Path $regKey -Name "Name"            -Value $providerName
Set-ItemProperty -Path $regKey -Name "Vendor"          -Value "OpenVCX"
Set-ItemProperty -Path $regKey -Name "FunctionLibrary" -Value (Join-Path $installDir $dllName)
Set-ItemProperty -Path $regKey -Name "ConfigApplication" -Value ""
Set-ItemProperty -Path $regKey -Name "APIVersion"      -Value "04.04"
Set-ItemProperty -Path $regKey -Name "ProductVersion"  -Value "04.04"

foreach ($p in $flags.GetEnumerator()) {
    New-ItemProperty -Path $regKey -Name $p.Key -Value $p.Value -PropertyType DWord -Force | Out-Null
}

Write-Host "installed: $(Join-Path $installDir $dllName)"
Write-Host "  sha256: $installedHash (matches build)"
Write-Host "registered: $regKey"
