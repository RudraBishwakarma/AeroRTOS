# AeroRTOS

> A software-based Real-Time Drone Flight Controller Simulator implementing a custom RTOS from scratch.

---

# Overview

AeroRTOS is a software-only embedded systems project that simulates the architecture of a professional drone flight controller while implementing a custom Real-Time Operating System (RTOS).

The project demonstrates how multiple real-time tasks execute concurrently to perform flight control, sensor fusion, navigation, safety monitoring, and system diagnostics without requiring physical hardware.

Instead of interacting with real sensors, AeroRTOS simulates drone dynamics and continuously executes the flight control pipeline while collecting runtime statistics and telemetry.

---

# Key Features

## Custom RTOS

- Preemptive Priority Scheduler
- Task Management
- Context Switching
- Tick-Based Scheduling
- Critical Sections
- Idle Task
- Software Timers

---

## Synchronization

- Binary Semaphores
- Message Queues
- Inter-Task Communication (IPC)

---

## Flight Control System

- Simulated IMU Sensor
- Extended Kalman Filter (EKF)
- Cascaded PID Flight Controller
- Quad-X Motor Mixer
- Navigation Module

---

## Safety System

- Watchdog Monitoring
- Health Monitor
- Battery Monitoring
- Communication Monitoring
- GPS Monitoring
- Automatic Failsafe Actions
- Auto Arm / Auto Disarm

---

## Performance Analysis

- CPU Usage Profiler
- Memory Profiler
- Deadline Analyzer
- Task Execution Statistics
- CSV Logger

---

# System Architecture

```text
                    AeroRTOS

        +--------------------------------------+
        |      Sensor Simulation Task          |
        +------------------+-------------------+
                           |
                           ▼
        +--------------------------------------+
        |    Extended Kalman Filter (EKF)      |
        +------------------+-------------------+
                           |
                           ▼
        +--------------------------------------+
        |   Cascaded PID Flight Controller     |
        +------------------+-------------------+
                           |
                           ▼
        +--------------------------------------+
        |      Quad-X Motor Mixer              |
        +------------------+-------------------+
                           |
                           ▼
        +--------------------------------------+
        |   Simulated Drone Dynamics           |
        +------------------+-------------------+
                           |
                           ▼
        +--------------------------------------+
        |   RTOS Monitoring & Profiling        |
        +------------------+-------------------+
                           |
                           ▼
        +--------------------------------------+
        |      CSV Telemetry Logger            |
        +--------------------------------------+
```

---

# RTOS Architecture

```text
                 Application Layer
                        │
                        ▼
              Flight Control Tasks
                        │
                        ▼
          -----------------------------
          |      AeroRTOS Kernel       |
          -----------------------------
          | Priority Scheduler         |
          | Context Switching          |
          | Task Management            |
          | Software Timers            |
          | Message Queues             |
          | Binary Semaphores          |
          -----------------------------
                        │
                        ▼
             Hardware Abstraction Layer
```

---

# Flight Control Pipeline

```text
Sensor Simulation
        │
        ▼
Sensor Task
        │
        ▼
State Estimation (EKF)
        │
        ▼
PID Flight Controller
        │
        ▼
Motor Mixer
        │
        ▼
Drone State Update
        │
        ▼
Telemetry Logger
```

---

# Communication Mechanisms

AeroRTOS implements several communication mechanisms commonly used in embedded real-time systems.

| Communication Mechanism | Purpose |
|-------------------------|---------|
| Message Queues | Inter-task communication |
| Binary Semaphores | Task synchronization |
| Critical Sections | Shared resource protection |
| Shared Memory | Internal kernel data exchange |
| CSV Logging | Runtime telemetry storage |
| UDP Socket Layer | Telemetry transport infrastructure |

---

# RTOS Tasks

| Task | Priority | Purpose |
|------|----------|----------|
| Sensor | Critical | Simulates IMU and sensor data |
| Control | Real-Time | Executes PID controller |
| Estimator | High | Runs Extended Kalman Filter |
| Safety | High | Performs failsafe monitoring |
| Watchdog | Normal | Monitors task execution |
| Performance Monitor | Low | CPU and memory profiling |
| Navigation | Normal | Navigation logic |
| Telemetry | Low | Logs flight telemetry |
| GCS Interface | Low | Handles telemetry interface |

---

# Runtime Features

During execution AeroRTOS performs:

- RTOS initialization
- Task creation
- Scheduler startup
- Sensor simulation
- State estimation
- PID control
- Navigation updates
- Safety monitoring
- Watchdog supervision
- CPU profiling
- Memory profiling
- Deadline analysis
- CSV telemetry logging

Example runtime output:

```text
Kernel initialized
Semaphores created
Message queues created
EKF Initialized
PID Controller Initialized
Motor Mixer Initialized
Watchdog Started
Health Monitor Started
CPU Profiler Started
Memory Profiler Started
CSV Logging Initialized
Scheduler Started
```

---

# Project Structure

```text
AeroRTOS
│
├── kernel/
│
├── estimation/
│
├── flight_controller/
│
├── communication/
│
├── middleware/
│
├── profiling/
│
├── safety/
│
├── simulation/
│
├── tools/
│
├── logs/
│
├── build/
│
├── Makefile
│
└── README.md
```

---

# Building

```bash
make
```

Run:

```bash
./build/aerortos.exe
```

---

# Telemetry Visualization

AeroRTOS continuously records simulation data into CSV log files.

A Python utility is provided to visualize the recorded telemetry.

Run:

```bash
python tools/flight_plotter.py
```

The plotter visualizes:

- Roll
- Pitch
- Yaw
- Altitude
- Velocity
- Battery Voltage
- CPU Usage
- Memory Usage
- Flight Path

using the telemetry generated by the simulator.

---

# Technologies Used

- C
- Python
- GCC
- Make
- Windows
- CSV Logging
- RTOS Design
- Embedded Systems
- Flight Control Algorithms

---

# Future Work

- ARM Cortex-M Port
- STM32 Support
- SPI Driver
- I²C Driver
- UART Driver
- CAN Bus Support
- Hardware Sensor Drivers
- SD Card Logging
- Hardware-in-the-Loop (HIL)
- Software-in-the-Loop (SIL)
- Multi-Vehicle Simulation

---

# Learning Outcomes

This project demonstrates practical implementation of:

- Real-Time Operating Systems
- Embedded Software Design
- Task Scheduling
- Context Switching
- Synchronization
- Sensor Fusion
- PID Control
- Flight Control Architecture
- Embedded Performance Analysis
- Software-Based System Simulation

---

# License

Licensed under the MIT License.
