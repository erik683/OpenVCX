# SPDX-License-Identifier: LGPL-3.0-only
# Builds the VCX Nano J2534 DLL (x86 for 32-bit diagnostic apps; x64 bonus)
# and the export, handoff, validation, transport, config, init, API, lifecycle and
# unload tests.
#
# PORTABLE: every path below is derived from this script's own location, so the
# whole OpenVCX directory can be copied anywhere (another machine, a USB stick)
# and built in place. No repository, no environment variables, no installer.
#
# Toolchain, in order of preference:
#   1. MSVC Build Tools, if installed on this machine
#   2. gcc on PATH
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File .\build_DLL_core.ps1
#   powershell -ExecutionPolicy Bypass -File .\build_DLL_core.ps1 -Archs x86,x64
#   powershell -ExecutionPolicy Bypass -File .\build_DLL_core.ps1 -Toolchain msvc
#   powershell -ExecutionPolicy Bypass -File .\build_DLL_core.ps1 -DllOnly -SkipTests
param(
    # Not [ValidateSet]: powershell.exe -File passes "x86,x64" as ONE argument
    # (it does not split on the comma the way -Command does), so the list is
    # normalised and checked below instead.
    [string[]]$Archs = @("x86"),
    [ValidateSet("auto","mingw","msvc")]
    [string]$Toolchain = "auto",
    [switch]$DllOnly,      # build just the DLL, no test harnesses
    [switch]$ResearchConfig, # enable bench overrides; output under build/research
    [switch]$SkipTests     # build the harnesses but do not run them
)

$ErrorActionPreference = "Stop"

$Archs = $Archs -split "," | ForEach-Object { $_.Trim() } | Where-Object { $_ }
foreach ($a in $Archs) {
    if ($a -notin @("x86","x64")) { throw "unknown arch '$a' - expected x86 and/or x64" }
}
if (-not $Archs) { throw "no arch given - expected x86 and/or x64" }

# --- portable layout: everything hangs off this script's directory ----------
$root = $PSScriptRoot
$dll  = Join-Path $root "dll"
if (-not (Test-Path (Join-Path $dll "api.c"))) {
    throw "sources not found at $dll - copy the whole OpenVCX directory, not just the scripts"
}

$srcList = @("api.c","device_vcx.c","pt_validate.c","vcx_transport.c") | ForEach-Object { Join-Path $dll $_ }
$def     = Join-Path $dll "j2534.def"

# --- toolchain discovery ----------------------------------------------------
function Get-Vcvars {
    # VCX_VCVARS: explicit vcvarsall.bat, set by build_DLL.ps1
    if ($env:VCX_VCVARS -and (Test-Path $env:VCX_VCVARS)) { return $env:VCX_VCVARS }
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $inst = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null | Select-Object -First 1
        if ($inst) {
            $cand = Join-Path $inst "VC\Auxiliary\Build\vcvarsall.bat"
            if (Test-Path $cand) { return $cand }
        }
    }
    $candidates = @(
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvarsall.bat",
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat",
        "$env:ProgramFiles\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat"
    )
    return ($candidates | Where-Object { Test-Path $_ } | Select-Object -First 1)
}

function Resolve-Toolchain([string]$arch) {
    # Returns @{ Kind = "mingw"|"msvc"; Gcc = <path>; Vcvars = <path> }
    if ($Toolchain -ne "mingw") {
        $vc = Get-Vcvars
        if ($vc) { return @{ Kind = "msvc"; Vcvars = $vc } }
    }
    if ($Toolchain -ne "msvc") {
        # a gcc already on PATH (must target the requested arch)
        $onPath = Get-Command gcc -ErrorAction SilentlyContinue
        if ($onPath) { return @{ Kind = "mingw"; Gcc = $onPath.Source } }
    }
    throw @"
no usable compiler for $arch.
  * install Visual Studio Build Tools with the C/C++ workload
  * or put a MinGW-w64 gcc targeting $arch on PATH
"@
}

# --- build fingerprint -------------------------------------------------------
# dll_build_info.h names the sources the DLL was built from: the git revision
# (with -dirty when the repository has uncommitted or untracked changes) and a SHA-256
# over the exact source bytes, which also identifies builds of copies with no git.
function Write-BuildInfo([string]$out) {
    $ErrorActionPreference = "Stop"       # a missing source hash must abort the build
    $files = @($srcList) + @($def) + @(Get-ChildItem (Join-Path $dll "*.h") | Sort-Object Name | ForEach-Object { $_.FullName })
    $manifest = ($files | ForEach-Object { "{0} {1}" -f (Get-FileHash $_ -Algorithm SHA256).Hash.ToLower(), (Split-Path $_ -Leaf) }) -join "`n"
    $sha = [Security.Cryptography.SHA256]::Create()
    $src = (($sha.ComputeHash([Text.Encoding]::UTF8.GetBytes($manifest)) | ForEach-Object { $_.ToString("x2") }) -join "").Substring(0, 12)
    $rev = "nogit"
    # A source snapshot nested inside another checkout must not inherit its ID.
    if ((Test-Path -LiteralPath (Join-Path $root ".git")) -and
        (Get-Command git -ErrorAction SilentlyContinue)) {
        try {
            # A source ZIP need not be a Git checkout. Only Git lookup is optional;
            # hashing source files above must never silently yield a partial ID.
            $ErrorActionPreference = "Continue"
            $head = & git -C $root rev-parse --short=12 HEAD 2>$null
            if ($LASTEXITCODE -eq 0 -and $head) {
                $dirty = & git -C $root status --porcelain -- . 2>$null
                $rev = "g$(($head | Select-Object -First 1).Trim())" + $(if ($dirty) { "-dirty" } else { "" })
            }
        } finally {
            $ErrorActionPreference = "Stop"
        }
    }
    $id = "$rev src:$src"
    Set-Content -Path (Join-Path $out "dll_build_info.h") -Encoding ascii -Value @(
        "/* Generated by build_DLL_core.ps1; do not edit. */",
        "#define DLL_BUILD_ID `"$id`""
    )
    Write-Host "  build id: $id"
}

function Write-DllHash([string]$path) {
    Write-Host "  sha256: $((Get-FileHash $path -Algorithm SHA256).Hash.ToLower())"
}

# --- per-arch build ---------------------------------------------------------
function Invoke-Native([string]$exe, [string[]]$argv, [string]$what) {
    & $exe @argv
    if ($LASTEXITCODE -ne 0) { throw "$what failed (exit $LASTEXITCODE)" }
}

function Build-Mingw([string]$arch, [string]$gcc, [string]$out, [string]$bits) {
    # Check the compiler on PATH targets the requested arch.
    # Naming the output "32" or "64" does not select GCC's target architecture.
    $target = & $gcc -dumpmachine
    if ($LASTEXITCODE -ne 0) { throw "cannot determine compiler target: $gcc" }
    $target = ($target -join "").Trim()
    $expected = if ($arch -eq "x86") { '^i[3-6]86-w64-mingw32$' } else { '^x86_64-w64-mingw32$' }
    if ($target -notmatch $expected) {
        throw "compiler '$gcc' targets '$target', incompatible with requested $arch; use the matching MinGW-w64 compiler or -Toolchain msvc"
    }

    # -static / -static-libgcc: the DLL is copied to Program Files on its own,
    # so it must not depend on libgcc_s / libwinpthread sitting beside it.
    # --kill-at: strip the stdcall @N decoration on x86 so the exports match the
    # plain names J2534 apps resolve via GetProcAddress (and j2534.def).
    # --enable-stdcall-fixup: j2534.def lists those plain names while the x86
    # objects carry _Name@N; ld matches them either way, this just silences the
    # per-export "resolving X by linking to X@N" warning it would otherwise emit.
    $common = @("-O2","-Wall","-D_CRT_SECURE_NO_WARNINGS","-I$dll","-static","-static-libgcc")
    if ($ResearchConfig) { $common += "-DVCX_RESEARCH_CONFIG" }
    $dllOut = Join-Path $out "vcxnano_pt$bits.dll"

    Write-Host "  cc: $gcc"
    Write-Host "  GCC version: $(& $gcc -dumpfullversion)"
    Write-BuildInfo $out
    Invoke-Native $gcc (@("-shared","-o",$dllOut) + $srcList + @($def) + $common + @("-DDLL_BUILD_INFO","-I$out","-Wl,--kill-at","-Wl,--enable-stdcall-fixup","-lwinmm")) "DLL link"
    Write-Host "  ok: $dllOut"
    Write-DllHash $dllOut
    if ($DllOnly) { return }

    Invoke-Native $gcc (@("-o",(Join-Path $out "dll_smoke.exe"),   (Join-Path $dll "dll_smoke.c"))   + $common) "dll_smoke"
    Invoke-Native $gcc (@("-o",(Join-Path $out "handoff_test.exe"),(Join-Path $dll "handoff_test.c"))+ $common) "handoff_test"
    Invoke-Native $gcc (@("-o",(Join-Path $out "validate_test.exe"),(Join-Path $dll "validate_test.c"),(Join-Path $dll "pt_validate.c")) + $common) "validate_test"
    Invoke-Native $gcc (@("-o",(Join-Path $out "transport_test.exe"),(Join-Path $dll "transport_test.c"),(Join-Path $dll "vcx_transport.c")) + $common) "transport_test"
    Invoke-Native $gcc (@("-o",(Join-Path $out "config_test.exe"), (Join-Path $dll "config_test.c"), (Join-Path $dll "pt_validate.c"), (Join-Path $dll "vcx_transport.c")) + $common + @("-lwinmm")) "config_test"
    Invoke-Native $gcc (@("-o",(Join-Path $out "init_test.exe"), (Join-Path $dll "init_test.c"), (Join-Path $dll "pt_validate.c"), (Join-Path $dll "vcx_transport.c")) + $common + @("-lwinmm")) "init_test"
    Invoke-Native $gcc (@("-o",(Join-Path $out "lifecycle_test.exe"), (Join-Path $dll "lifecycle_test.c"), (Join-Path $dll "pt_validate.c"), (Join-Path $dll "vcx_transport.c")) + $common + @("-lwinmm")) "lifecycle_test"
    Invoke-Native $gcc (@("-o",(Join-Path $out "api_test.exe"), (Join-Path $dll "api_test.c"), (Join-Path $dll "pt_validate.c"), (Join-Path $dll "vcx_transport.c")) + $common + @("-lwinmm")) "api_test"
    Invoke-Native $gcc (@("-shared","-o",(Join-Path $out "unload_test.dll"), (Join-Path $dll "unload_test_dll.c"), (Join-Path $dll "pt_validate.c"), (Join-Path $dll "vcx_transport.c"), (Join-Path $dll "unload_test.def")) + $common + @("-lwinmm")) "unload_test DLL"
    Invoke-Native $gcc (@("-o",(Join-Path $out "unload_test.exe"), (Join-Path $dll "unload_test.c")) + $common) "unload_test"
}

function Build-Msvc([string]$arch, [string]$vcvars, [string]$out, [string]$bits) {
    # cmd.exe cannot use a UNC cwd; build in a temp dir.
    $work = Join-Path $env:TEMP ("openvcx_build_{0}_{1}" -f $arch, [Guid]::NewGuid().ToString("N"))
    New-Item -ItemType Directory -Force $work | Out-Null
    $srcs = ($srcList | ForEach-Object { "`"$_`"" }) -join " "

    $researchDefine = if ($ResearchConfig) { "/DVCX_RESEARCH_CONFIG" } else { "" }
    Write-BuildInfo $out
    $lines = @(
        '@echo off',
        "cd /d `"$work`"",
        "call `"$vcvars`" $arch >nul 2>nul",
        'if errorlevel 1 exit /b 1',
        'echo MSVC tools version: %VCToolsVersion%',
        "cl /nologo /W3 /O2 /MT $researchDefine /D_CRT_SECURE_NO_WARNINGS /DDLL_BUILD_INFO /I`"$dll`" /I`"$out`" /LD $srcs /Fe`"$out\vcxnano_pt$bits.dll`" /link /DEF:`"$def`" winmm.lib",
        'if errorlevel 1 exit /b 1'
    )
    if (-not $DllOnly) {
        $lines += @(
            "cl /nologo /W3 /O2 /MT $researchDefine /D_CRT_SECURE_NO_WARNINGS `"$dll\dll_smoke.c`" /Fe`"$out\dll_smoke.exe`"",
            'if errorlevel 1 exit /b 1',
            "cl /nologo /W3 /O2 /MT $researchDefine /D_CRT_SECURE_NO_WARNINGS `"$dll\handoff_test.c`" /Fe`"$out\handoff_test.exe`"",
            'if errorlevel 1 exit /b 1',
            "cl /nologo /W3 /O2 /MT $researchDefine /D_CRT_SECURE_NO_WARNINGS /I`"$dll`" `"$dll\validate_test.c`" `"$dll\pt_validate.c`" /Fe`"$out\validate_test.exe`"",
            'if errorlevel 1 exit /b 1',
            "cl /nologo /W3 /O2 /MT $researchDefine /D_CRT_SECURE_NO_WARNINGS /I`"$dll`" `"$dll\transport_test.c`" `"$dll\vcx_transport.c`" /Fe`"$out\transport_test.exe`"",
            'if errorlevel 1 exit /b 1',
            "cl /nologo /W3 /O2 /MT $researchDefine /D_CRT_SECURE_NO_WARNINGS /I`"$dll`" `"$dll\config_test.c`" `"$dll\pt_validate.c`" `"$dll\vcx_transport.c`" /Fe`"$out\config_test.exe`" /link winmm.lib",
            'if errorlevel 1 exit /b 1',
            "cl /nologo /W3 /O2 /MT $researchDefine /D_CRT_SECURE_NO_WARNINGS /I`"$dll`" `"$dll\init_test.c`" `"$dll\pt_validate.c`" `"$dll\vcx_transport.c`" /Fe`"$out\init_test.exe`" /link winmm.lib",
            'if errorlevel 1 exit /b 1',
            "cl /nologo /W3 /O2 /MT $researchDefine /D_CRT_SECURE_NO_WARNINGS /I`"$dll`" `"$dll\lifecycle_test.c`" `"$dll\pt_validate.c`" `"$dll\vcx_transport.c`" /Fe`"$out\lifecycle_test.exe`" /link winmm.lib",
            'if errorlevel 1 exit /b 1',
            "cl /nologo /W3 /O2 /MT $researchDefine /D_CRT_SECURE_NO_WARNINGS /I`"$dll`" `"$dll\api_test.c`" `"$dll\pt_validate.c`" `"$dll\vcx_transport.c`" /Fe`"$out\api_test.exe`" /link winmm.lib",
            'if errorlevel 1 exit /b 1',
            "cl /nologo /W3 /O2 /MT $researchDefine /D_CRT_SECURE_NO_WARNINGS /I`"$dll`" /LD `"$dll\unload_test_dll.c`" `"$dll\pt_validate.c`" `"$dll\vcx_transport.c`" /Fe`"$out\unload_test.dll`" /link /DEF:`"$dll\unload_test.def`" winmm.lib",
            'if errorlevel 1 exit /b 1',
            "cl /nologo /W3 /O2 /MT $researchDefine /D_CRT_SECURE_NO_WARNINGS `"$dll\unload_test.c`" /Fe`"$out\unload_test.exe`"",
            'if errorlevel 1 exit /b 1'
        )
    }
    $cmdFile = Join-Path $work "build.cmd"
    Set-Content -Path $cmdFile -Value ($lines -join "`r`n") -Encoding ascii
    try {
        & cmd /c $cmdFile
        if ($LASTEXITCODE -ne 0) { throw "MSVC build failed for $arch" }
    } finally {
        $resolvedWork = [IO.Path]::GetFullPath($work)
        $resolvedTemp = [IO.Path]::GetFullPath($env:TEMP).TrimEnd('\')
        if ([IO.Path]::GetDirectoryName($resolvedWork) -eq $resolvedTemp -and
            [IO.Path]::GetFileName($resolvedWork) -match '^openvcx_build_(x86|x64)_[0-9a-f]{32}$') {
            Remove-Item -LiteralPath $resolvedWork -Recurse -Force
        }
    }
    Write-Host "  ok: $out\vcxnano_pt$bits.dll"
    Write-DllHash "$out\vcxnano_pt$bits.dll"
}

foreach ($arch in $Archs) {
    $bits = if ($arch -eq "x86") { "32" } else { "64" }
    $out  = if ($ResearchConfig) { Join-Path $root "build\research\$arch" } else { Join-Path $root "build\$arch" }
    New-Item -ItemType Directory -Force $out | Out-Null

    $tc = Resolve-Toolchain $arch
    Write-Host "=== Building $arch ($($tc.Kind)) ==="
    if ($tc.Kind -eq "mingw") { Build-Mingw $arch $tc.Gcc $out $bits }
    else                      { Build-Msvc  $arch $tc.Vcvars $out $bits }

    if (-not $DllOnly -and -not $SkipTests) {
        # Self-checks that need no hardware. dll_smoke / handoff_test do, so
        # they are built but not run here.
        foreach ($t in @("validate_test.exe","transport_test.exe","config_test.exe","init_test.exe","lifecycle_test.exe","api_test.exe")) {
            $exe = Join-Path $out $t
            Write-Host "--- $t ---"
            & $exe
            if ($LASTEXITCODE -ne 0) { throw "$t failed (exit $LASTEXITCODE)" }
        }
        & (Join-Path $out "unload_test.exe") (Join-Path $out "unload_test.dll")
        if ($LASTEXITCODE -ne 0) { throw "unload_test failed (exit $LASTEXITCODE)" }
        & (Join-Path $out "dll_smoke.exe") (Join-Path $out "vcxnano_pt$bits.dll") exports-only
        if ($LASTEXITCODE -ne 0) { throw "DLL export/load check failed (exit $LASTEXITCODE)" }
    }
}

Write-Host ""
Write-Host "Next: install (elevated PowerShell):  .\install_DLL.ps1 -Arch $($Archs[0])"
