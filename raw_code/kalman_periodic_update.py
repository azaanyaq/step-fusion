import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import find_peaks

DATA_DIR = "/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_6"

### Step detection ###

df_acc = pd.read_csv(f"{DATA_DIR}/Accelerometer.csv")
df_acc["magnitude"] = (df_acc["x"] ** 2 + df_acc["y"] ** 2 + df_acc["z"] ** 2) ** 0.5
mag = df_acc["magnitude"].to_numpy()
t_acc = df_acc["seconds_elapsed"].to_numpy()

sampling_rate_hz = 1 / np.mean(np.diff(t_acc))
min_step_gap_samples = int(0.4 * sampling_rate_hz)
peak_indices, _ = find_peaks(mag, prominence=1.0, distance=min_step_gap_samples)
step_times = t_acc[peak_indices]

### Gyroscope ###

df_gyr = pd.read_csv(f"{DATA_DIR}/Gyroscope.csv")
t_gyr = df_gyr["seconds_elapsed"].to_numpy()
yaw_rate = df_gyr["z"].to_numpy()
dt = np.diff(t_gyr, prepend=t_gyr[0])
mean_dt = np.mean(dt[1:])
heading_gyro_rad = np.cumsum(yaw_rate * dt)

### Magnetometer ###

df_mag = pd.read_csv(f"{DATA_DIR}/Magnetometer.csv")
t_mag = df_mag["seconds_elapsed"].to_numpy()
mag_x = df_mag["x"].to_numpy()
mag_y = df_mag["y"].to_numpy()
heading_mag_rad = np.unwrap(-np.arctan2(mag_y, mag_x))
heading_mag_interp = np.interp(t_gyr, t_mag, heading_mag_rad)

### Noise estimates - same windows as demo_6_analysis.py ###

stationary_mask = t_gyr <= 1.5
Q = np.var(yaw_rate[stationary_mask]) * mean_dt**2

straight_mask = (t_mag >= 57.0) & (t_mag <= 58.5)
R_BASE = np.var(heading_mag_rad[straight_mask])

### Which gyro sample indices equate to a step ###

step_update_indices = set(np.searchsorted(t_gyr, step_times))


def run_kalman_periodic(Q, R):
    """Predict every gyro sample (fast); only update at step events (slow)."""
    theta = heading_mag_interp[0]
    P = R
    theta_history = [theta]

    for i in range(1, len(t_gyr)):
        # Predict always happens - same as before
        theta_pred = theta + yaw_rate[i] * dt[i]
        P_pred = P + Q

        if i in step_update_indices:
            # Only correct toward magnetometer when a step was detected.
            K = P_pred / (P_pred + R)
            theta = theta_pred + K * (heading_mag_interp[i] - theta_pred)
            P = (1 - K) * P_pred
        else:
            # Carry prediction forward if no available data
            theta = theta_pred
            P = P_pred

        theta_history.append(theta)

    return np.array(theta_history)


def build_path(heading_array, heading_time_array):
    step_headings = np.interp(step_times, heading_time_array, heading_array)
    x, y = [0.0], [0.0]
    for i in range(len(step_headings)):
        x.append(x[i] + 0.73 * np.cos(step_headings[i]))
        y.append(y[i] + 0.73 * np.sin(step_headings[i]))
    return np.array(x), np.array(y)


def closing_distance(x, y):
    return np.hypot(x[-1], y[-1])


heading_kf_periodic = run_kalman_periodic(Q, R_BASE)
heading_gyro_rad_aligned = heading_gyro_rad - heading_gyro_rad[0] + heading_kf_periodic[0]

x_raw, y_raw = build_path(heading_gyro_rad_aligned, t_gyr)
x_kf, y_kf = build_path(heading_kf_periodic, t_gyr)

print(f"Raw gyro-only closing distance: {closing_distance(x_raw, y_raw):.3f} m")
print(f"Kalman (per-step update) closing distance: {closing_distance(x_kf, y_kf):.3f} m")

print("\nR sweep (per-step update):")
for mult in [0.1, 0.5, 1, 5, 20, 50, 100]:
    h = run_kalman_periodic(Q, R_BASE * mult)
    x_s, y_s = build_path(h, t_gyr)
    print(f"  R x{mult:<6}: closing distance = {closing_distance(x_s, y_s):.3f} m")

plt.plot(x_raw, y_raw, alpha=0.6, label="raw gyro-only")
plt.plot(x_kf, y_kf, linewidth=1.5, label="Kalman (per-step update)")
plt.title("Dead-reckoning path: per-step update Kalman filter (demo_6)")
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.gca().set_aspect("equal")
plt.legend()
plt.grid(True)

plt.show()
