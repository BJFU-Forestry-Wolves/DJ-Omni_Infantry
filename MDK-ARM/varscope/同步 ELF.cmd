@echo off
rem ============================================================
rem  Sync ELF  --  Keil .axf  ->  VarScope .elf
rem  ASCII only on purpose: cmd.exe mis-parses UTF-8 Chinese,
rem  so every Chinese message is printed by sync-elf.ps1 instead.
rem ============================================================
setlocal
title Sync ELF for VarScope
cd /d "%~dp0"

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0sync-elf.ps1"
set "RC=%ERRORLEVEL%"

echo.
if "%RC%"=="0" (
    echo [OK] Done. In VarScope just press "Connect" again.
) else (
    echo [FAILED] See the red lines above.
)
echo.
echo Press any key to close...
pause >nul
endlocal