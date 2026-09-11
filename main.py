import matplotlib.pyplot as plt

from pdr_pipeline import (
    build_path,
    closing_distance,
    compute_gyro_heading,
    detect_steps,
    estimate_noise,
    load_recording,
    run_kalman_filter,
)

R_MULTIPLIER = 1000 # Constant multiplier on the R value, so that filter trusts magnetometer data less

# Recordings organised into a dictionary. "label" is just the plot heading -
# edit it freely, it's separate from the folder name used to load the data.
RECORDINGS = {
    "demo_4": {"label": "demo_4 - indoors and short sample", "stationary_window_s": (0.0, 3.5), "straight_window_s": (7.0, 9.3)},
    "demo_5": {"label": "demo_5 - outdoors and short sample", "stationary_window_s": (0.0, 5.0), "straight_window_s": (10.0, 13.0)},
    "demo_6": {"label": "demo_6 - outdoors and long sample", "stationary_window_s": (0.0, 1.5), "straight_window_s": (57.0, 58.5)},
}


def evaluate(name, config):
    data_dir = f"data/raw/{name}"
    rec = load_recording(data_dir)

    step_times = detect_steps(rec)
    Q, R_base = estimate_noise(rec, config["stationary_window_s"], config["straight_window_s"])

    # Compute headings
    heading_gyro_rad = compute_gyro_heading(rec)
    heading_kf_rad = run_kalman_filter(rec, step_times, Q, R_base * R_MULTIPLIER)
    heading_gyro_rad_aligned = heading_gyro_rad - heading_gyro_rad[0] + heading_kf_rad[0]

    x_raw, y_raw = build_path(step_times, heading_gyro_rad_aligned, rec["t_gyr"])
    x_kf, y_kf = build_path(step_times, heading_kf_rad, rec["t_gyr"])

    raw_cd = closing_distance(x_raw, y_raw)
    kf_cd = closing_distance(x_kf, y_kf)
    improvement_pct = (raw_cd - kf_cd) / raw_cd * 100

    return {
        "label": config["label"],
        "improvement_pct": improvement_pct,
        "x_raw": x_raw, "y_raw": y_raw,
        "x_kf": x_kf, "y_kf": y_kf,
    }


results = {name: evaluate(name, config) for name, config in RECORDINGS.items()}

fig, axes = plt.subplots(1, len(RECORDINGS), figsize=(6 * len(RECORDINGS), 5))

for ax, (name, r) in zip(axes, results.items()):
    ax.plot(r["x_raw"], r["y_raw"], marker="o", alpha=0.6, label="raw gyro-only")
    ax.plot(r["x_kf"], r["y_kf"], marker="o", linewidth=2, label="Kalman-fused")
    ax.set_title(f"{r['label']}\n{r['improvement_pct']:+.1f}% closing-distance improvement")
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.set_aspect("equal")
    ax.legend()
    ax.grid(True)

plt.tight_layout()
plt.show()