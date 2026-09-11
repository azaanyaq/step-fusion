import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import find_peaks

DATA_DIR = "/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_6"

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
print(f"Detected {len(peak_indices)} steps (actual: 150)")

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
mag_z = df_mag["z"].to_numpy()
heading_mag_rad = np.unwrap(-np.arctan2(mag_y, mag_x))
heading_mag_interp = np.interp(t_gyr, t_mag, heading_mag_rad)

field_magnitude = np.sqrt(mag_x**2 + mag_y**2 + mag_z**2)
print(f"Field magnitude: min={field_magnitude.min():.2f} max={field_magnitude.max():.2f} "
      f"mean={field_magnitude.mean():.2f} std={field_magnitude.std():.3f} uT")

### Noise estimates - windows picked by eye for THIS (much longer) recording ###

STATIONARY_WINDOW_S = (0.0, 1.5)
stationary_mask = t_gyr <= STATIONARY_WINDOW_S[1]
Q = np.var(yaw_rate[stationary_mask]) * mean_dt**2

STRAIGHT_WINDOW_S = (57.0, 58.5) # Flattest 1.5s window found by search
straight_mask = (t_mag >= STRAIGHT_WINDOW_S[0]) & (t_mag <= STRAIGHT_WINDOW_S[1])
R_BASE = np.var(heading_mag_rad[straight_mask])

print(f"Q = {Q:.8f} rad^2/step   R = {R_BASE:.6f} rad^2")

### Kalman filter - heading fusion, same as kalman_heading.py ###


def run_kalman(Q, R):
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


heading_kf_rad = run_kalman(Q, R_BASE)

### Position dead-reckoning - same as kalman_position.py ###

STEP_LENGTH_M = 0.73


def build_path(heading_array, heading_time_array):
    step_headings = np.interp(step_times, heading_time_array, heading_array)
    x, y = [0.0], [0.0]
    for i in range(len(step_headings)):
        x.append(x[i] + STEP_LENGTH_M * np.cos(step_headings[i]))
        y.append(y[i] + STEP_LENGTH_M * np.sin(step_headings[i]))
    return np.array(x), np.array(y)


def closing_distance(x, y):
    return np.hypot(x[-1], y[-1])


heading_gyro_rad_aligned = heading_gyro_rad - heading_gyro_rad[0] + heading_kf_rad[0]

x_raw, y_raw = build_path(heading_gyro_rad_aligned, t_gyr)
x_kf, y_kf = build_path(heading_kf_rad, t_gyr)

raw_closing = closing_distance(x_raw, y_raw)
kf_closing = closing_distance(x_kf, y_kf)
print(f"Raw gyro-only closing distance: {raw_closing:.3f} m")
print(f"Kalman-fused (base R) closing distance: {kf_closing:.3f} m")

### R sweep - same as r_sweep.py ###

R_MULTIPLIERS = [0.1, 0.5, 1, 5, 20, 50, 100, 500, 2000]
print("\nR sweep:")
for mult in R_MULTIPLIERS:
    heading_kf_sweep = run_kalman(Q, R_BASE * mult)
    x_sweep, y_sweep = build_path(heading_kf_sweep, t_gyr)
    print(f"  R x{mult:<6}: closing distance = {closing_distance(x_sweep, y_sweep):.3f} m")

### Plots ###

fig, axes = plt.subplots(1, 3, figsize=(16, 5))

axes[0].plot(t_gyr, np.degrees(heading_gyro_rad_aligned), label="gyro only", alpha=0.6)
axes[0].plot(t_gyr, np.degrees(heading_mag_interp), label="magnetometer only", alpha=0.5)
axes[0].plot(t_gyr, np.degrees(heading_kf_rad), label="Kalman fused", color="black", linewidth=1.5)
axes[0].set_title("Heading comparison (109s, ~5.5 loops)")
axes[0].set_xlabel("Time elapsed (s)")
axes[0].set_ylabel("Heading (degrees)")
axes[0].legend()
axes[0].grid(True)

axes[1].plot(x_raw, y_raw, alpha=0.6, label="raw gyro-only")
axes[1].plot(x_kf, y_kf, linewidth=1.5, label="Kalman-fused")
axes[1].set_title("Dead-reckoning path")
axes[1].set_xlabel("X position (m)")
axes[1].set_ylabel("Y position (m)")
axes[1].set_aspect("equal")
axes[1].legend()
axes[1].grid(True)

axes[2].plot(t_mag, field_magnitude)
axes[2].axhline(field_magnitude.mean(), color="gray", linestyle="--")
axes[2].set_title("Magnetometer field magnitude over time")
axes[2].set_xlabel("Time elapsed (s)")
axes[2].set_ylabel("Field magnitude (uT)")
axes[2].grid(True)

plt.tight_layout()
plt.show()
