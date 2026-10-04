@echo off
rem SPDX-License-Identifier: LGPL-3.0-only
rem Double-clickable install. The provider key lives in HKLM, so this needs
rem administrator rights: if it was not started elevated it relaunches itself
rem elevated (UAC prompt) and re-runs with the same arguments.
rem Portable: install_DLL.ps1 is taken from this file's own directory.
rem
rem Usage: install.cmd [-Arch x86^|x64^|both] [-Port COM5] [-Log] [-Uninstall]
rem With no arguments, installs both architectures.
setlocal
rem Avoid inheriting incompatible PowerShell 7 modules through cmd.exe.
set "PSModulePath="
set "PS1=%~dp0install_DLL.ps1"
net session >nul 2>&1
if not errorlevel 1 goto :elevated

echo Requesting administrator rights...
rem -NoExit keeps the elevated window open so its output stays readable.
powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process -Verb RunAs -FilePath 'powershell.exe' -ArgumentList (@('-NoProfile','-ExecutionPolicy','Bypass','-NoExit','-File',('\"' + $env:PS1 + '\"')) + ('%*' -split ' +' | Where-Object { $_ }))"
exit /b 0

:elevated
powershell -NoProfile -ExecutionPolicy Bypass -File "%PS1%" %*
set RC=%errorlevel%
if not "%RC%"=="0" (
    echo.
    echo INSTALL FAILED ^(exit %RC%^)
)
echo.
pause
endlocal & exit /b %RC%
