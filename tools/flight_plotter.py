#!/usr/bin/env python3
"""AeroRTOS live flight data plotter."""

from collections import deque
from pathlib import Path
import time

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pandas as pd

DATA_FILE = Path("flight_data.csv")
HEALTH_FILE = Path("health_data.csv")
UPDATE_INTERVAL_MS = 100
MAX_POINTS = 200

series = {
    "timestamp": deque(maxlen=MAX_POINTS),
    "roll": deque(maxlen=MAX_POINTS),
    "pitch": deque(maxlen=MAX_POINTS),
    "yaw": deque(maxlen=MAX_POINTS),
    "altitude": deque(maxlen=MAX_POINTS),
    "motor1": deque(maxlen=MAX_POINTS),
    "motor2": deque(maxlen=MAX_POINTS),
    "motor3": deque(maxlen=MAX_POINTS),
    "motor4": deque(maxlen=MAX_POINTS),
}


def read_latest_csv(path):
    if not path.exists() or path.stat().st_size == 0:
        return None
    try:
        data = pd.read_csv(path)
    except (pd.errors.EmptyDataError, pd.errors.ParserError, OSError):
        return None
    if data.empty:
        return None
    return data


def append_latest(row):
    required = series.keys()
    if any(name not in row for name in required):
        return
    for name in required:
        series[name].append(row[name])


def update_plot(_frame, axes):
    flight_df = read_latest_csv(DATA_FILE)
    health_df = read_latest_csv(HEALTH_FILE)

    if flight_df is not None:
        append_latest(flight_df.iloc[-1])

    ax1, ax2, ax3, ax4, ax5, ax6 = axes
    for axis in axes:
        axis.clear()

    timestamps = list(series["timestamp"])
    if timestamps:
        ax1.set_title("Attitude")
        ax1.set_xlabel("Time (ms)")
        ax1.set_ylabel("Angle (deg)")
        ax1.plot(timestamps, [v * 57.2958 for v in series["roll"]], "r-", label="Roll")
        ax1.plot(timestamps, [v * 57.2958 for v in series["pitch"]], "g-", label="Pitch")
        ax1.plot(timestamps, [v * 57.2958 for v in series["yaw"]], "b-", label="Yaw")
        ax1.legend(loc="upper right")
        ax1.grid(True)

        ax2.set_title("Altitude")
        ax2.set_xlabel("Time (ms)")
        ax2.set_ylabel("Meters")
        ax2.plot(timestamps, list(series["altitude"]), "b-", linewidth=2)
        ax2.axhline(y=10.0, color="r", linestyle="--", label="Target")
        ax2.legend(loc="upper right")
        ax2.grid(True)

        ax3.set_title("Motor Outputs")
        ax3.set_xlabel("Time (ms)")
        ax3.set_ylabel("Output")
        ax3.plot(timestamps, list(series["motor1"]), "r-", label="M1")
        ax3.plot(timestamps, list(series["motor2"]), "g-", label="M2")
        ax3.plot(timestamps, list(series["motor3"]), "b-", label="M3")
        ax3.plot(timestamps, list(series["motor4"]), "y-", label="M4")
        ax3.set_ylim(0.0, 1.0)
        ax3.legend(loc="upper right")
        ax3.grid(True)

    ax4.set_title("3D Position")
    ax4.set_xlabel("X (m)")
    ax4.set_ylabel("Y (m)")
    ax4.set_zlabel("Z (m)")
    if flight_df is not None and {"pos_x", "pos_y", "pos_z"}.issubset(flight_df.columns):
        trail = flight_df.tail(50)
        ax4.plot(trail["pos_x"], trail["pos_y"], trail["pos_z"], "b-", linewidth=1)
        last = trail.iloc[-1]
        ax4.scatter([last["pos_x"]], [last["pos_y"]], [last["pos_z"]], c="red", s=50)

    ax5.set_title("CPU Usage")
    ax5.set_xlabel("Time (ms)")
    ax5.set_ylabel("CPU %")
    ax5.set_ylim(0, 100)
    if health_df is not None and {"timestamp", "cpu_usage"}.issubset(health_df.columns):
        health_tail = health_df.tail(100)
        ax5.plot(health_tail["timestamp"], health_tail["cpu_usage"], "b-")
    ax5.grid(True)

    ax6.set_title("Battery Voltage")
    ax6.set_xlabel("Time (ms)")
    ax6.set_ylabel("Voltage (V)")
    if health_df is not None and {"timestamp", "battery_voltage"}.issubset(health_df.columns):
        health_tail = health_df.tail(100)
        ax6.plot(health_tail["timestamp"], health_tail["battery_voltage"], "g-", linewidth=2)
        ax6.axhline(y=10.5, color="r", linestyle="--", label="Min")
        ax6.legend(loc="upper right")
    ax6.grid(True)

    plt.tight_layout()


def main():
    print("[PLOTTER] Starting AeroRTOS live visualization")
    print("[PLOTTER] Waiting for flight_data.csv and health_data.csv")
    print("[PLOTTER] Press Ctrl+C to stop")

    fig = plt.figure(figsize=(14, 10))
    axes = [
        fig.add_subplot(3, 2, 1),
        fig.add_subplot(3, 2, 2),
        fig.add_subplot(3, 2, 3),
        fig.add_subplot(3, 2, 4, projection="3d"),
        fig.add_subplot(3, 2, 5),
        fig.add_subplot(3, 2, 6),
    ]
    ani = animation.FuncAnimation(fig, update_plot, fargs=(axes,), interval=UPDATE_INTERVAL_MS, cache_frame_data=False)
    fig._aerortos_animation = ani
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n[PLOTTER] Stopped")


if __name__ == "__main__":
    main()