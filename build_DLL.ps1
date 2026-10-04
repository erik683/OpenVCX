# SPDX-License-Identifier: LGPL-3.0-only
# Finds a C compiler for each requested arch, asks for its path if none is
# found, then builds with build_DLL_core.ps1.
#
# Looks for, in order:
#   1. MSVC (Visual Studio / Build Tools) via vswhere and the default folders
#   2. MinGW-w64 gcc on PATH, then in the usual MSYS2 / MinGW folders
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File .\build_DLL.ps1
#   powershell -ExecutionPolicy Bypass -File .\build_DLL.ps1 -Archs x86,x64

param(
    [string[]]$Archs = @("x86")
)

$ErrorActionPreference = "Stop"

$Archs = $Archs -split "," | ForEach-Object { $_.Trim() } | Where-Object { $_ }
foreach ($a in $Archs) {
    if ($a -notin @("x86","x64")) { throw "unknown arch '$a' - expected x86 and/or x64" }
}
if (-not $Archs) { throw "no arch given - expected x86 and/or x64" }

function Find-Vcvars {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $inst = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null | Select-Object -First 1
        if ($inst) {
            $cand = Join-Path $inst "VC\Auxiliary\Build\vcvarsall.bat"
            if (Test-Path $cand) { return $cand }
        }
    }
    $candidates = foreach ($root in @(${env:ProgramFiles(x86)}, $env:ProgramFiles)) {
        foreach ($ver in @("18", "2022", "2019")) {
            foreach ($ed in @("BuildTools", "Community", "Professional", "Enterprise")) {
                "$root\Microsoft Visual Studio\$ver\$ed\VC\Auxiliary\Build\vcvarsall.bat"
            }
        }
    }
    return ($candidates | Where-Object { Test-Path $_ } | Select-Object -First 1)
}

function Get-GccTarget([string]$gcc) {
    $t = & $gcc -dumpmachine 2>$null
    if ($LASTEXITCODE -ne 0) { return $null }
    return "$t".Trim()
}

function Test-GccArch([string]$gcc, [string]$arch) {
    $t = Get-GccTarget $gcc
    if (-not $t) { return $false }
    if ($arch -eq "x86") { return $t -match '^i[3-6]86-w64-mingw32$' }
    return $t -match '^x86_64-w64-mingw32$'
}

function Find-Gcc([string]$arch) {
    $sub = if ($arch -eq "x86") { "mingw32" } else { "mingw64" }
    $candidates = @()
    $onPath = Get-Command gcc -All -ErrorAction SilentlyContinue
    if ($onPath) { $candidates += $onPath | ForEach-Object { $_.Source } }
    $candidates += @(
        "C:\msys64\$sub\bin\gcc.exe",
        "C:\$sub\bin\gcc.exe",
        "$env:ProgramData\$sub\bin\gcc.exe",
        "$env:ProgramFiles\mingw-w64\$sub\bin\gcc.exe",
        "${env:ProgramFiles(x86)}\mingw-w64\$sub\bin\gcc.exe"
    )
    foreach ($c in $candidates) {
        if ((Test-Path $c) -and (Test-GccArch $c $arch)) { return $c }
    }
    return $null
}

function Read-CompilerPath([string]$arch) {
    Write-Host "No compiler found for $arch."
    Write-Host "Enter the full path to vcvarsall.bat (Visual Studio) or to a MinGW-w64 gcc.exe"
    Write-Host "that targets $arch, or press Enter to cancel."
    while ($true) {
        $p = (Read-Host "Path").Trim().Trim('"')
        if (-not $p) { throw "no compiler for $arch - cancelled" }
        if (-not (Test-Path $p -PathType Leaf)) { Write-Host "Not found: $p"; continue }
        $name = [IO.Path]::GetFileName($p)
        if ($name -ieq "vcvarsall.bat") { return @{ Kind = "msvc"; Path = $p } }
        if ($name -ieq "gcc.exe") {
            if (Test-GccArch $p $arch) { return @{ Kind = "mingw"; Path = $p } }
            Write-Host "That gcc targets '$(Get-GccTarget $p)', not $arch."
            continue
        }
        Write-Host "Expected vcvarsall.bat or gcc.exe."
    }
}

$buildCore = Join-Path $PSScriptRoot "build_DLL_core.ps1"

foreach ($arch in $Archs) {
    $tc = $null
    $vc = Find-Vcvars
    if ($vc) { $tc = @{ Kind = "msvc"; Path = $vc } }
    if (-not $tc) {
        $gcc = Find-Gcc $arch
        if ($gcc) { $tc = @{ Kind = "mingw"; Path = $gcc } }
    }
    if (-not $tc) { $tc = Read-CompilerPath $arch }

    Write-Host "[$arch] using $($tc.Kind): $($tc.Path)"
    $savedPath = $env:PATH
    $savedVcvars = $env:VCX_VCVARS
    try {
        if ($tc.Kind -eq "msvc") {
            $env:VCX_VCVARS = $tc.Path
        } else {
            $env:PATH = (Split-Path $tc.Path) + ";" + $env:PATH
        }
        & powershell -NoProfile -ExecutionPolicy Bypass -File $buildCore -Archs $arch -Toolchain $tc.Kind
        if ($LASTEXITCODE -ne 0) { throw "build failed for $arch (exit $LASTEXITCODE)" }
    } finally {
        $env:PATH = $savedPath
        $env:VCX_VCVARS = $savedVcvars
    }
}
