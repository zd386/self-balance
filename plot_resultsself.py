"""
Reads balance_log.csv (produced by main.py) and plots pitch angle over time.
This is the "control system response curve" graph -- put it in your README.
"""

import csv
import os

import matplotlib.pyplot as plt

LOG_PATH = os.path.join(os.path.dirname(__file__), "balance_log.csv")


def load_log(path):
    times, pitches, outputs = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time"]))
            pitches.append(float(row["pitch_rad"]))
            outputs.append(float(row["pid_output"]))
    return times, pitches, outputs


def main():
    if not os.path.exists(LOG_PATH):
        print(f"No log found at {LOG_PATH}. Run main.py first.")
        return

    times, pitches, outputs = load_log(LOG_PATH)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    ax1.plot(times, pitches, color="#2f6fdb")
    ax1.axhline(0, color="gray", linestyle="--", linewidth=1)
    ax1.set_ylabel("Pitch (rad)")
    ax1.set_title("Self-Balancing Robot: Tilt Angle Over Time")
    ax1.grid(True, alpha=0.3)

    ax2.plot(times, outputs, color="#d64545")
    ax2.set_ylabel("Wheel velocity cmd")
    ax2.set_xlabel("Time (s)")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()

    out_path = os.path.join(os.path.dirname(__file__), "balance_response.png")
    fig.savefig(out_path, dpi=150)
    print(f"Saved plot to {out_path}")

    plt.show()


if __name__ == "__main__":
    main()
