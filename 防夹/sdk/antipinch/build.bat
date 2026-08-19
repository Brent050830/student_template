@echo off
chcp 65001 >nul
setlocal EnableDelayedExpansion
cd /d "%~dp0"

REM ============================================================
REM  build.bat - Seat Anti-Pinch SDK build script
REM  Tool : C:\mingw64\bin\mingw32-make.exe  (fixed path)
REM Usage: build.bat [data_csv]   (default: ../data/test.csv)
REM  Runs the two make actions: run, plot
REM ============================================================

set "MAKE=C:\mingw64\bin\mingw32-make.exe"
set "FAIL=0"

REM Data CSV argument (default if not provided)
if "%~1"=="" (
    set "DATA_CSV=../data/test.csv"
) else (
    set "DATA_CSV=%~1"
)

if not exist "%MAKE%" (
    echo [build.bat] Error: mingw32-make.exe not found at %MAKE%
    echo [build.bat] Please install MinGW-w64 to C:\mingw64 or edit MAKE path.
    exit /b 1
)

echo ============================================================
echo  Seat Anti-Pinch SDK - build [run / plot]
echo  Make tool: %MAKE%
echo  Dataset:  %DATA_CSV%
echo ============================================================
echo.

REM --- Action 1/2: run ---
echo [1/2] make run   -- evaluate on dataset
"%MAKE%" run DATA_CSV=%DATA_CSV%
if !errorlevel! neq 0 (
    echo [1/2] run   -- FAIL  exit=!errorlevel!
    set /a FAIL+=1
) else (
    echo [1/2] run   -- PASS
)
echo.

REM --- Action 2/2: plot ---
echo [2/2] make plot  -- generate result plot
"%MAKE%" plot DATA_CSV=%DATA_CSV%
if !errorlevel! neq 0 (
    echo [2/2] plot  -- FAIL  exit=!errorlevel!
    set /a FAIL+=1
) else (
    echo [2/2] plot  -- PASS
)
echo.

echo ============================================================
if %FAIL% equ 0 (
    echo  All two actions completed successfully.
) else (
    echo  Done. %FAIL% action^(s^) reported failure.
)
echo ============================================================
exit /b %FAIL%
