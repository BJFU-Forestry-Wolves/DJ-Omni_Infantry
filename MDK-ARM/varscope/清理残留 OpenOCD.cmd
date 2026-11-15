@echo off
rem ============================================================
rem  Kill stray openocd.exe processes that hold the debug probe.
rem  Fixes: "CMSIS-DAP command CMD_INFO failed" caused by a
rem  previous VarScope session that did not exit cleanly.
rem  ASCII only on purpose (cmd.exe mis-parses UTF-8 Chinese).
rem ============================================================
setlocal
title Clean stray OpenOCD
cd /d "%~dp0"

echo Listing openocd.exe processes...
echo.
tasklist /FI "IMAGENAME eq openocd.exe" /FO TABLE 2>nul | find /I "openocd.exe" >nul
if errorlevel 1 (
    echo   [OK] No openocd.exe running. Nothing to clean.
    goto :end
)

tasklist /FI "IMAGENAME eq openocd.exe" /FO TABLE
echo.
echo Killing them...
taskkill /F /IM openocd.exe >nul 2>&1
if errorlevel 1 (
    echo   [FAILED] Could not kill them. Try Task Manager as admin.
) else (
    echo   [OK] All openocd.exe killed.
    echo   Now go back to VarScope and press "Connect" again.
)

:end
echo.
echo Press any key to close...
pause >nul
endlocal