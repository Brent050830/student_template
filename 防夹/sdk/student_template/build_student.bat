@echo off
REM build_student.bat - Build and run student solution
REM Usage: build_student.bat
REM
REM Requires: GCC (MinGW) in PATH

echo === Building student_solution.c ===

REM -- Compile (link directly against DLL) --
set PATH=%PATH%;..\lib
gcc -Wall -Wextra -O2 -std=c11 -I../include -o run.exe student_solution.c -L../lib -l:competition_mock.dll -lm
if errorlevel 1 (
    echo ERROR: Build failed.
    exit /b 1
)

REM -- Run --
echo.
echo === Running ===
run.exe
echo.

echo === Done ===
