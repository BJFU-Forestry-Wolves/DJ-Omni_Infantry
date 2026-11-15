@echo off
rem ============================================================
rem  Run VarScope from source (developer use only).
rem  Normal users should just double-click the packaged
rem  VarScope.exe in the portable folder instead.
rem  ASCII only on purpose (cmd.exe mis-parses UTF-8 Chinese).
rem ============================================================
setlocal
title VarScope from source
cd /d "%~dp0"

set "SRC=C:\Users\ghko5\Documents\Github\VarScope-src"
set "PYTHONPATH=%SRC%"

if not exist "%SRC%\launch_varscope.py" (
    echo [ERROR] Source tree not found: %SRC%
    pause
    exit /b 1
)

if not exist "%~dp0gimbal_task.elf" (
    echo [WARN] gimbal_task.elf missing. Run sync-elf.ps1 first.
)

python "%SRC%\launch_varscope.py" --elf "%~dp0gimbal_task.elf" --cfg "%~dp0stm32f407-daplink.cfg"
if errorlevel 1 pause
endlocal