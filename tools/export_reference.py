"""
Exports the Python pipeline's outputs as CSVs (test_data/reference/<recording>/)
so the C++ tests can compare against them. Run from the repo root.
"""

import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pdr_pipeline import (  # noqa: E402
    STEP_LENGTH_M,
    build_path,
    closing_distance,
    compute_gyro_heading,
    compute_magnetometer_heading,
    detect_steps,
    estimate_noise,
    load_recording,
    run_kalman_filter,
)

# Copied from main.py (importing it would run its plots), keep in sync
R_MULTIPLIER = 1000
RECORDINGS = {
    "demo_4": {"stationary_window_s": (0.0, 3.5), "straight_window_s": (7.0, 9.3)},
    "demo_5": {"stationary_window_s": (0.0, 5.0), "straight_window_s": (10.0, 13.0)},
    "demo_6": {"stationary_window_s": (0.0, 1.5), "straight_window_s": (57.0, 58.5)},
}

FLOAT_FORMAT = "%.17g" # Enough digits to round trip a double exactly
OUT_ROOT = "test_data/reference"


def export(name, config):
    rec = load_recording(f"data/raw/{name}")

    # C++ uses the latest mag sample instead of np.interp, only identical if timestamps match
    if not (np.array_equal(rec["t_acc"], rec["t_gyr"]) and np.array_equal(rec["t_gyr"], rec["t_mag"])):
        raise RuntimeError(f"{name}: sensor timestamps differ - golden KF comparison would not be exact")

    step_times = detect_steps(rec)
    step_indices = np.searchsorted(rec["t_gyr"], step_times)

    Q, R_base = estimate_noise(rec, config["stationary_window_s"], config["straight_window_s"])
    R = R_base * R_MULTIPLIER

    heading_gyro = compute_gyro_heading(rec)
    heading_mag = compute_magnetometer_heading(rec)
    heading_kf = run_kalman_filter(rec, step_times, Q, R)
    heading_gyro_aligned = heading_gyro - heading_gyro[0] + heading_kf[0]  # same alignment as main.py

    x_raw, y_raw = build_path(step_times, heading_gyro_aligned, rec["t_gyr"])
    x_kf, y_kf = build_path(step_times, heading_kf, rec["t_gyr"])
    raw_cd = closing_distance(x_raw, y_raw)
    kf_cd = closing_distance(x_kf, y_kf)

    out_dir = f"{OUT_ROOT}/{name}"
    os.makedirs(out_dir, exist_ok=True)

    pd.DataFrame({
        "t": rec["t_gyr"],
        "gyro_heading": heading_gyro,
        "mag_heading": heading_mag,
        "kf_heading": heading_kf,
    }).to_csv(f"{out_dir}/heading.csv", index=False, float_format=FLOAT_FORMAT)

    pd.DataFrame({"index": step_indices, "t": step_times}).to_csv(
        f"{out_dir}/steps.csv", index=False, float_format=FLOAT_FORMAT)

    pd.DataFrame({"raw_x": x_raw, "raw_y": y_raw, "kf_x": x_kf, "kf_y": y_kf}).to_csv(
        f"{out_dir}/path.csv", index=False, float_format=FLOAT_FORMAT)

    summary = {
        "n_samples": len(rec["t_gyr"]),
        "n_steps": len(step_times),
        "step_length_m": STEP_LENGTH_M,
        "Q": Q,
        "R_base": R_base,
        "R_multiplier": R_MULTIPLIER,
        "R": R,
        "theta0": heading_kf[0],
        "P0": R,
        "raw_closing_m": raw_cd,
        "kf_closing_m": kf_cd,
        "improvement_pct": (raw_cd - kf_cd) / raw_cd * 100,
    }
    pd.DataFrame(list(summary.items()), columns=["key", "value"]).to_csv(
        f"{out_dir}/summary.csv", index=False, float_format=FLOAT_FORMAT)

    print(f"{name}: {len(rec['t_gyr'])} samples, {len(step_times)} steps, "
          f"raw {raw_cd:.3f} m, kf {kf_cd:.3f} m -> {out_dir}/")


if __name__ == "__main__":
    for name, config in RECORDINGS.items():
        export(name, config)
