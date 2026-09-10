#!/usr/bin/env python3
"""Plot signals from the latest voice-coil log against time.

Reads the most recent ``data/voice_coil_log_*.csv`` file and draws three
stacked axes sharing the time axis:

* top:    actual / target / demand current (A)  -- with show/hide checkboxes
* middle: ai1_value
* bottom: ai2_value

Usage:
    python scripts/plot_voice_coil_log.py [path/to/log.csv]

With no argument the newest log in ``data/`` is used.
"""

from __future__ import annotations

import csv
import glob
import os
import sys

import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(REPO_ROOT, "data")

# One entry per axis:
#   (y-axis label, checkboxes?, {trace label: (csv column, colour)})
AXES = [
    (
        "current (A)",
        True,
        {
            "actual current": ("actual_current_A", "tab:blue"),
            "target current": ("target_current_A", "tab:orange"),
            "demand current": ("demand_current_A", "tab:green"),
        },
    ),
    (
        "ai1_value (V)",
        False,
        {"ai1_value": (("ai1_value_V", "ai1_value"), "tab:brown")},
    ),
    (
        "ai2_value (V)",
        False,
        {"ai2_value": (("ai2_value_V", "ai2_value"), "tab:cyan")},
    ),
]

ALL_SIGNALS = {label: spec for _, _, group in AXES for label, spec in group.items()}


def latest_log() -> str:
    logs = glob.glob(os.path.join(DATA_DIR, "voice_coil_log_*.csv"))
    if not logs:
        sys.exit(f"no voice_coil_log_*.csv files found in {DATA_DIR}")
    return max(logs, key=os.path.getmtime)


def read_log(path: str):
    time_s: list[float] = []
    series: dict[str, list[float]] = {label: [] for label in ALL_SIGNALS}
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        reader.fieldnames = [name.strip() for name in (reader.fieldnames or [])]
        for row in reader:
            row = {k.strip(): v for k, v in row.items()}
            try:
                t = float(row["time_s"])
            except (KeyError, TypeError, ValueError):
                continue
            time_s.append(t)
            for label, (column, _) in ALL_SIGNALS.items():
                names = (column,) if isinstance(column, str) else column
                value = float("nan")
                for name in names:
                    try:
                        value = float(row[name])
                        break
                    except (KeyError, TypeError, ValueError):
                        continue
                series[label].append(value)
    return time_s, series


def add_checkboxes(fig, rect, signals, lines, ax):
    """Attach a CheckButtons group at figure coords ``rect`` for ``signals``."""
    labels = list(signals)
    colours = [signals[l][1] for l in labels]
    rax = fig.add_axes(rect)
    rax.set_frame_on(False)
    check = CheckButtons(
        rax,
        labels,
        actives=[True] * len(labels),
        label_props={"color": colours},
        frame_props={"edgecolor": colours},
        check_props={"facecolor": colours},
    )

    def toggle(label: str) -> None:
        line = lines[label]
        line.set_visible(not line.get_visible())
        visible = [lines[l] for l in labels if lines[l].get_visible()]
        legend = ax.get_legend()
        if visible:
            ax.legend(handles=visible, loc="upper right")
        elif legend is not None:
            legend.remove()
        # Rescale the y-axis to fit only the visible traces.
        ax.relim(visible_only=True)
        ax.autoscale_view(scalex=False, scaley=True)
        fig.canvas.draw_idle()

    check.on_clicked(toggle)
    return check


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else latest_log()
    time_s, series = read_log(path)
    if not time_s:
        sys.exit(f"no usable rows in {path}")

    n = len(AXES)
    fig, axarr = plt.subplots(n, 1, sharex=True, figsize=(11, 10))
    fig.subplots_adjust(top=0.88, bottom=0.07, hspace=0.55)
    fig.suptitle(f"Voice-coil log — {os.path.basename(path)}", y=0.99)

    lines: dict = {}
    checks = []
    for i, (ax, (ylabel, has_checks, group)) in enumerate(zip(axarr, AXES)):
        for label, (column, colour) in group.items():
            (line,) = ax.plot(time_s, series[label], label=label, color=colour, lw=1.0)
            lines[label] = line
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
        if i == n - 1:
            ax.set_xlabel("time_s (s)")

        if has_checks:
            # Checkbox group just above this axis.
            pos = ax.get_position()
            rect = [pos.x0, pos.y1 + 0.012, 0.6, 0.055]
            checks.append(add_checkboxes(fig, rect, group, lines, ax))

    fig._checkboxes = checks  # keep refs alive

    plt.show()


if __name__ == "__main__":
    main()
