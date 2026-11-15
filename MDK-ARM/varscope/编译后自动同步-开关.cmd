@echo off
rem ============================================================
rem  Toggle: auto-sync ELF after every Keil build.
rem  ASCII only on purpose (cmd.exe mis-parses UTF-8 Chinese);
rem  Chinese text is printed by keil-autosync.ps1 instead.
rem ============================================================
setlocal
title Auto-sync ELF after Keil build - toggle
cd /d "%~dp0"

echo ------------------------------------------------------------
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0keil-autosync.ps1" -Status
echo ------------------------------------------------------------
echo.
echo   [1] Enable  auto-sync after build
echo   [2] Disable auto-sync after build
echo   [3] Exit without changes
echo.
set /p CHOICE=Choose 1 / 2 / 3 then press Enter: 

if "%CHOICE%"=="1" goto :enable
if "%CHOICE%"=="2" goto :disable
echo.
echo No changes made.
goto :end

:enable
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0keil-autosync.ps1" -Enable
goto :show

:disable
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0keil-autosync.ps1" -Disable
goto :show

:show
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0keil-autosync.ps1" -Status
echo.
echo If it did not take effect, restart Keil so it re-reads the project.

:end
echo.
echo Press any key to close...
pause >nul
endlocal