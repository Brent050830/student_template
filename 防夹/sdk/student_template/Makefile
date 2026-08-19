# =============================================================================
# Makefile - BEV Battery Preheat Decision Algorithm Competition (Student Version)
#
# Edit ONLY: student_solution.c
#
# Usage:
#   mingw32-make          # build and run
#   mingw32-make build    # build only
#   mingw32-make run      # run only (auto-copies DLL next to run.exe)
#   mingw32-make clean    # remove build artifacts
#
# Note: No sh.exe on this box, so mingw32-make runs recipes by direct
#       CreateProcess (no shell). cmd-internal commands (copy/del) are wrapped
#       in `cmd /c`. The runtime DLL is copied next to run.exe so Windows loads
#       it without needing ../lib on PATH.
# =============================================================================

CC       = gcc
CFLAGS   = -Wall -Wextra -O2 -std=c11
LDFLAGS  = -lm
INCLUDE  = -I../include
LIBDIR   = -L../lib
DLL      = competition_mock.dll
DLL_SRC  = ../lib/$(DLL)
TARGET   = run.exe
STUDENT  = student_solution.c

.PHONY: all build run clean

all: build run

build: $(TARGET) $(DLL)

$(TARGET): $(STUDENT)
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $(STUDENT) $(LIBDIR) -l:$(DLL) $(LDFLAGS)

$(DLL): $(DLL_SRC)
	cmd /c "copy /Y ..\lib\$(DLL) ."

run: $(TARGET) $(DLL)
	$(TARGET)

clean:
	-@cmd /c "del /Q $(TARGET) *.o $(DLL) 2>nul"
