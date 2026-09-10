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

### Gyroscope - raw heading kept for comparison ###

df_gyr = pd.read_csv(f"{DATA_DIR}/Gyroscope.csv")
t_gyr = df_gyr["seconds_elapsed"].to_numpy()
yaw_rate = df_gyr["z"].to_numpy()

dt = np.diff(t_gyr, prepend=t_gyr[0])
mean_dt = np.mean(dt[1:])
heading_gyro_rad = np.cumsum(yaw_rate * dt)  # raw, uncorrected

### Magnetometer ###

df_mag = pd.read_csv(f"{DATA_DIR}/Magnetometer.csv")
t_mag = df_mag["seconds_elapsed"].to_numpy()
mag_x = df_mag["x"].to_numpy()
mag_y = df_mag["y"].to_numpy()
heading_mag_rad = np.unwrap(-np.arctan2(mag_y, mag_x))

### Noise estimates - same windows as noise_estimation.py ###

STRAIGHT_WINDOW_S = (7.0, 9.3)
straight_mask = (t_mag >= STRAIGHT_WINDOW_S[0]) & (t_mag <= STRAIGHT_WINDOW_S[1])
R = np.var(heading_mag_rad[straight_mask])

STATIONARY_WINDOW_S = (0.0, 3.5)
stationary_mask = t_gyr <= STATIONARY_WINDOW_S[1]
Q = np.var(yaw_rate[stationary_mask]) * mean_dt**2

heading_mag_interp = np.interp(t_gyr, t_mag, heading_mag_rad)

### Kalman filter - heading fusion, same as kalman_heading.py ###

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

heading_kf_rad = np.array(theta_history)

### Position dead-reckoning: build a path from any heading array + its own timeline ###

STEP_LENGTH_M = 0.73
step_times = t_acc[peak_indices]  # seconds, from the step detector's timeline


def build_path(heading_array, heading_time_array):
    step_headings = np.interp(step_times, heading_time_array, heading_array)
    x, y = [0.0], [0.0]
    for i in range(len(step_headings)):
        x.append(x[i] + STEP_LENGTH_M * np.cos(step_headings[i]))
        y.append(y[i] + STEP_LENGTH_M * np.sin(step_headings[i]))
    return x, y


# heading_gyro_rad starts at an arbitrary 0 (whatever direction you faced at
# t=0), while heading_kf_rad starts at the magnetometer's real-world-anchored
# value. Shift the raw heading to start at that same reference point before
# building its path - otherwise the two paths are drawn in different rotated
# coordinate frames, which distorts the comparison regardless of how good or
# bad the filter actually is.
heading_gyro_rad_aligned = heading_gyro_rad - heading_gyro_rad[0] + heading_kf_rad[0]

# Same logic as interp.py, called twice - once per heading source - instead of
# duplicating the loop, since the only thing that changes is which heading
# array (and its matching timeline) gets passed in.
x_raw, y_raw = build_path(heading_gyro_rad_aligned, t_gyr)
x_kf, y_kf = build_path(heading_kf_rad, t_gyr)

plt.plot(x_raw, y_raw, marker="o", label="raw gyro-only DR path", alpha=0.6)
plt.plot(x_kf, y_kf, marker="o", label="Kalman-fused DR path", linewidth=2)
plt.title("Dead-reckoning path: raw gyro-only vs. Kalman-fused heading")
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.gca().set_aspect("equal")
plt.legend()
plt.grid(True)

plt.show()
