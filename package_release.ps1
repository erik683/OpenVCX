# SPDX-License-Identifier: LGPL-3.0-only
# Build a standalone source snapshot, test both DLLs, and package only public files.
param(
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]*$')]
    [string]$Version = "local"
)
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$name = "OpenVCX-$Version"
$dist = Join-Path $root "dist"
$zip = Join-Path $dist "$name.zip"
if (Test-Path -LiteralPath $zip) { throw "$zip already exists; choose a new -Version." }

# An allowlist keeps captures, toolchains and unrelated local files out of releases.
$files = @(
    "README.md", "CONFIGURATION.md", "CONTRIBUTING.md", "LICENSE", "COPYING",
    "build.cmd", "build_DLL.ps1", "build_DLL_core.ps1",
    "install.cmd", "install_DLL.ps1", "package_release.ps1"
)
$files += @(
    "api.c", "api_test.c", "config_test.c", "device.h", "device_vcx.c",
    "dll_smoke.c", "handoff_test.c", "ini_config.h", "init_test.c",
    "j2534.def", "j2534.h", "j2534_defs.h", "lifecycle_test.c", "pt_validate.c",
    "pt_validate.h", "pt_wire.h", "transport_test.c", "unload_test.c",
    "unload_test.def", "unload_test_dll.c", "validate_test.c",
    "vcx_transport.c", "vcx_transport.h"
) | ForEach-Object { "dll/$_" }

$work = Join-Path $root ("build\release-" + [Guid]::NewGuid().ToString("N"))
$source = Join-Path $work "source"
$package = Join-Path $work $name
foreach ($file in $files) {
    $target = Join-Path $source $file
    New-Item -ItemType Directory -Force (Split-Path $target) | Out-Null
    Copy-Item -LiteralPath (Join-Path $root $file) -Destination $target
}

# Build the snapshot rather than a changing working tree. Its source fingerprint
# identifies the included files; no .git directory or private build path is shipped.
$log = Join-Path $work "build.log"
& powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $source "build_DLL.ps1") -Archs "x86,x64" *> $log
if ($LASTEXITCODE -ne 0) { throw "Release build/checks failed. See $log" }
foreach ($file in $files) {
    $target = Join-Path $package $file
    New-Item -ItemType Directory -Force (Split-Path $target) | Out-Null
    Copy-Item -LiteralPath (Join-Path $source $file) -Destination $target
}
foreach ($arch in @("x86", "x64")) {
    $bits = if ($arch -eq "x86") { "32" } else { "64" }
    $target = Join-Path $package "bin\$arch"
    New-Item -ItemType Directory -Force $target | Out-Null
    Copy-Item -LiteralPath (Join-Path $source "build\$arch\vcxnano_pt$bits.dll") -Destination (Join-Path $target "OpenVCX$bits.dll")
}
$buildDetails = Get-Content -LiteralPath $log |
    Where-Object { $_ -match 'MSVC tools version:|GCC version:|build id:' }
@(
    "OpenVCX $Version", "Built (UTC): $([DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ'))",
    "Source: included alongside the binaries; build ID uses its source fingerprint.",
    "Checks: seven offline suites and DLL load/14-export check per architecture.",
    "No hardware or installation test performed.", "", $buildDetails
) | Set-Content -LiteralPath (Join-Path $package "release-info.txt") -Encoding utf8

$hashes = Get-ChildItem -LiteralPath $package -Recurse -File | Sort-Object FullName | ForEach-Object {
    $relative = $_.FullName.Substring($package.Length + 1).Replace('\', '/')
    "{0}  {1}" -f (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLower(), $relative
}
$hashes | Set-Content -LiteralPath (Join-Path $package "SHA256SUMS.txt") -Encoding ascii
New-Item -ItemType Directory -Force $dist | Out-Null
Compress-Archive -LiteralPath $package -DestinationPath $zip -CompressionLevel Optimal
"{0}  {1}" -f (Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash.ToLower(), "$name.zip" |
    Set-Content -LiteralPath "$zip.sha256" -Encoding ascii
Write-Host "Package: $zip"
Write-Host "Build log: $log"
