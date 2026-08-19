# Seat Anti-Pinch SDK - Student Build Makefile
# Compatible with Windows PowerShell + MinGW-w64 gcc
#
# Usage: make run      -> build runner and execute against public dataset
#        make plot     -> run runner + generate result plot PNG
#        make clean    -> clean build artifacts
#
# Override dataset: make run DATA_CSV=../data/validate.csv

CC      = gcc
CFLAGS  = -Wall -Wextra -std=c11 -O2
LDFLAGS = -lm
SDK_DIR = ..

SRC     = student_solution.c
OBJ     = student_solution.o

DATA_CSV    ?= ../data/test.csv
OUTPUT_CSV  = eval_output.csv
PLOT_PNG    = 防夹检测_*.png

.PHONY: all run plot clean

all: run

$(OBJ): $(SRC)
	$(CC) $(CFLAGS) -I$(SDK_DIR)/include -c $< -o $@

# Runner: student code + pre-built runtime main + SDK libs
# (relink when student code OR any SDK lib changes)
SDK_LIBS = $(SDK_DIR)/lib/seat_runner_main.o $(SDK_DIR)/lib/libseatruntime.a $(SDK_DIR)/lib/libseatdata.a

seat_runner.exe: $(OBJ) $(SDK_LIBS)
	$(CC) $(CFLAGS) -o $@ $(OBJ) $(SDK_LIBS) $(LDFLAGS)

run: seat_runner.exe
	seat_runner.exe --data $(DATA_CSV)

# Plot: run runner with --output, then call Python plot script
plot: seat_runner.exe
	-seat_runner.exe --data $(DATA_CSV) --output $(OUTPUT_CSV)
	python plot_result.py --input $(DATA_CSV) --output $(OUTPUT_CSV)

clean:
	-@del /Q $(OBJ) seat_runner.exe $(OUTPUT_CSV) $(PLOT_PNG) 2>nul
