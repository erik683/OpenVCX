@echo off
rem SPDX-License-Identifier: LGPL-3.0-only
rem Double-clickable build. Portable: paths come from %~dp0, so this works
rem wherever the OpenVCX directory was copied to.
setlocal
rem Avoid inheriting incompatible PowerShell 7 modules through cmd.exe.
set "PSModulePath="
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_DLL.ps1" %*
set RC=%errorlevel%
if not "%RC%"=="0" (
    echo.
    echo BUILD FAILED ^(exit %RC%^)
)
echo.
pause
endlocal & exit /b %RC%
