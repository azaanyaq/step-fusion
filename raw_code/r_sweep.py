import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import find_peaks

DATA_DIR = "/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_4"

### Step detection (accelerometer) - same as find_peaks_scipy.py ###

df_acc = pd.read_csv(f"{DATA_DIR}/Accelerometer.csv")
df_acc["magnitude"] = (df_acc["x"] ** 2 + df_acc["y"] ** 2 + df_acc["z"] ** 2) ** 0.5
mag = df_acc["magnitude"].to_numpy()
t_acc = df_acc["seconds_elapsed"].to_numpy()

sampling_rate_hz = 1 / np.mean(np.diff(t_acc))
HEIGHT_THRESHOLD = 1.0
MIN_STEP_GAP_S = 0.4
min_step_gap_samples = int(MIN_STEP_GAP_S * sampling_rate_hz)
peak_indices, _properties = find_peaks(mag, prominence=HEIGHT_THRESHOLD, distance=min_step_gap_samples)
step_times = t_acc[peak_indices]

### Gyroscope ###

df_gyr = pd.read_csv(f"{DATA_DIR}/Gyroscope.csv")
t_gyr = df_gyr["seconds_elapsed"].to_numpy()
yaw_rate = df_gyr["z"].to_numpy()

dt = np.diff(t_gyr, prepend=t_gyr[0])
mean_dt = np.mean(dt[1:])
heading_gyro_rad = np.cumsum(yaw_rate * dt) # Raw and uncorrected

### Magnetometer ###

df_mag = pd.read_csv(f"{DATA_DIR}/Magnetometer.csv")
t_mag = df_mag["seconds_elapsed"].to_numpy()
mag_x = df_mag["x"].to_numpy()
mag_y = df_mag["y"].to_numpy()
heading_mag_rad = np.unwrap(-np.arctan2(mag_y, mag_x))
heading_mag_interp = np.interp(t_gyr, t_mag, heading_mag_rad)

### Base noise estimates - same windows as noise_estimation.py ###

STRAIGHT_WINDOW_S = (7.0, 9.3)
straight_mask = (t_mag >= STRAIGHT_WINDOW_S[0]) & (t_mag <= STRAIGHT_WINDOW_S[1])
R_BASE = np.var(heading_mag_rad[straight_mask])

STATIONARY_WINDOW_S = (0.0, 3.5)
stationary_mask = t_gyr <= STATIONARY_WINDOW_S[1]
Q = np.var(yaw_rate[stationary_mask]) * mean_dt**2

STEP_LENGTH_M = 0.73


def run_kalman(Q, R):
    """Predict/update heading fusion loop, identical logic to kalman_heading.py."""
    theta = heading_mag_interp[0]
    P = R
    theta_history = [theta]
    for i in range(1, len(t_gyr)):
        theta_pred = theta + yaw_rate[i] * dt[i]
        P_pred = P + Q

        K = P_pred / (P_pred + R)
        theta = theta_pred + K * (heading_mag_interp[i] - theta_pred)
        P = (1 - K) * P_pred

        theta_history.append(theta)
    return np.array(theta_history)


def build_path(heading_array, heading_time_array):
    """Identical logic to kalman_position.py's build_path."""
    step_headings = np.interp(step_times, heading_time_array, heading_array)
    x, y = [0.0], [0.0]
    for i in range(len(step_headings)):
        x.append(x[i] + STEP_LENGTH_M * np.cos(step_headings[i]))
        y.append(y[i] + STEP_LENGTH_M * np.sin(step_headings[i]))
    return np.array(x), np.array(y)


def closing_distance(x, y):
    """Straight-line distance from the path's start (0,0) back to its end point."""
    return np.hypot(x[-1], y[-1])


x_raw, y_raw = build_path(heading_gyro_rad, t_gyr)
raw_closing = closing_distance(x_raw, y_raw)

R_MULTIPLIERS = [1, 5, 20, 50, 100, 500, 2000]
kf_closing_distances = []

for mult in R_MULTIPLIERS:
    heading_kf_rad = run_kalman(Q, R_BASE * mult)
    x_kf, y_kf = build_path(heading_kf_rad, t_gyr)
    kf_closing_distances.append(closing_distance(x_kf, y_kf))

print(f"Raw gyro-only closing distance: {raw_closing:.3f} m")
for mult, cd in zip(R_MULTIPLIERS, kf_closing_distances):
    print(f"R x{mult:<5}: closing distance = {cd:.3f} m")

plt.plot(R_MULTIPLIERS, kf_closing_distances, marker="o", label="Kalman-fused")
plt.axhline(raw_closing, color="gray", linestyle="--", label="raw gyro-only (baseline)")
plt.xscale("log")
plt.xlabel("R multiplier (log scale)")
plt.ylabel("Closing distance (m)")
plt.title("Effect of trusting the magnetometer less (higher R) on loop-closing error")
plt.legend()
plt.grid(True)

plt.show()
