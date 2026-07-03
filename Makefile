# AeroRTOS Makefile
CC := gcc
BUILD_DIR := build
TARGET := $(BUILD_DIR)/aerortos.exe

CPPFLAGS := -I. -I./kernel -I./estimation -I./flight_controller -I./communication -I./middleware -I./safety -I./profiling -I./simulation -D_GNU_SOURCE
CFLAGS := -Wall -Wextra -O2 -g -Wno-address-of-packed-member
LDFLAGS :=
LDLIBS := -lwinmm -lws2_32 -lm

SRC := $(wildcard kernel/*.c) \
       $(wildcard estimation/*.c) \
       $(wildcard flight_controller/*.c) \
       $(wildcard flight_controller/dynamics/*.c) \
       $(wildcard flight_controller/sensors/*.c) \
       $(wildcard communication/*.c) \
       $(wildcard middleware/*.c) \
       $(wildcard safety/*.c) \
       $(wildcard profiling/*.c) \
       $(wildcard simulation/*.c) \
       $(wildcard tools/*.c) \
       main.c

# rtos_task.c is an older companion module; task management is implemented in rtos_kernel.c.
SRC := $(filter-out kernel/rtos_task.c,$(SRC))
OBJ := $(patsubst %.c,$(BUILD_DIR)/%.o,$(SRC))
DEP := $(OBJ:.o=.d)

.PHONY: all clean run debug test plot mavlink demo print-sources

all: $(TARGET)

$(TARGET): $(OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)
	@echo ""
	@echo "AeroRTOS build complete: $(TARGET)"
	@echo ""

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
	@echo "Clean complete"

run: $(TARGET)
	@echo "Running AeroRTOS..."
	./$(TARGET)

debug: $(TARGET)
	gdb -ex "break kernel_schedule" -ex "run" ./$(TARGET)

test: $(TARGET)
	@echo "Running test for 15 seconds..."
	timeout 15 ./$(TARGET) || true


plot: $(TARGET)
	@echo "Starting AeroRTOS with CSV logging for live plotting..."
	@echo "Run in another terminal: python tools/flight_plotter.py"
	./$(TARGET)

mavlink: $(TARGET)
	@echo "Starting AeroRTOS with MAVLink UDP telemetry..."
	@echo "Connect Mission Planner or QGroundControl to UDP port 14550"
	./$(TARGET)

demo: $(TARGET)
	@echo "Starting AeroRTOS full visualization demo..."
	@echo "1. This terminal: AeroRTOS running"
	@echo "2. Another terminal: python tools/flight_plotter.py"
	@echo "3. GCS: connect UDP 127.0.0.1:14550"
	./$(TARGET)
print-sources:
	@printf '%s\n' $(SRC)

-include $(DEP)