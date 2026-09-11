import numpy as np
import pandas as pd
from scipy.signal import find_peaks

STEP_LENGTH_M = 0.73

# Windows picked by eye per-recording
RECORDINGS = {
    "demo_4": {
        "stationary_window_s": (0.0, 3.5),
        "straight_window_s": (7.0, 9.3),
    },
    "demo_5": {
        "stationary_window_s": (0.0, 5.0),
        "straight_window_s": (10.0, 13.0),
    },
    "demo_6": {
        "stationary_window_s": (0.0, 1.5),
        "straight_window_s": (57.0, 58.5),
    },
}


def load_recording(name):
    data_dir = f"/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/{name}"

    df_acc = pd.read_csv(f"{data_dir}/Accelerometer.csv")
    df_acc["magnitude"] = (df_acc["x"] ** 2 + df_acc["y"] ** 2 + df_acc["z"] ** 2) ** 0.5
    mag = df_acc["magnitude"].to_numpy()
    t_acc = df_acc["seconds_elapsed"].to_numpy()

    sampling_rate_hz = 1 / np.mean(np.diff(t_acc))
    min_step_gap_samples = int(0.4 * sampling_rate_hz)
    peak_indices, _ = find_peaks(mag, prominence=1.0, distance=min_step_gap_samples)
    step_times = t_acc[peak_indices]

    df_gyr = pd.read_csv(f"{data_dir}/Gyroscope.csv")
    t_gyr = df_gyr["seconds_elapsed"].to_numpy()
    yaw_rate = df_gyr["z"].to_numpy()
    dt = np.diff(t_gyr, prepend=t_gyr[0])
    mean_dt = np.mean(dt[1:])
    heading_gyro_rad = np.cumsum(yaw_rate * dt)

    df_mag = pd.read_csv(f"{data_dir}/Magnetometer.csv")
    t_mag = df_mag["seconds_elapsed"].to_numpy()
    mag_x = df_mag["x"].to_numpy()
    mag_y = df_mag["y"].to_numpy()
    heading_mag_rad = np.unwrap(-np.arctan2(mag_y, mag_x))
    heading_mag_interp = np.interp(t_gyr, t_mag, heading_mag_rad)

    return {
        "step_times": step_times,
        "t_gyr": t_gyr,
        "yaw_rate": yaw_rate,
        "dt": dt,
        "mean_dt": mean_dt,
        "heading_gyro_rad": heading_gyro_rad,
        "t_mag": t_mag,
        "heading_mag_rad": heading_mag_rad,
        "heading_mag_interp": heading_mag_interp,
    }


def estimate_noise(rec, stationary_window_s, straight_window_s):
    stationary_mask = rec["t_gyr"] <= stationary_window_s[1]
    Q = np.var(rec["yaw_rate"][stationary_mask]) * rec["mean_dt"] ** 2

    straight_mask = (rec["t_mag"] >= straight_window_s[0]) & (rec["t_mag"] <= straight_window_s[1])
    R = np.var(rec["heading_mag_rad"][straight_mask])
    return Q, R


def run_kalman_periodic(rec, Q, R):

    """
    Predict every gyro sample; only update (pull toward magnetometer) at
    detected step events. See kalman_periodic_update.py for the reasoning -
    updating this infrequently, rather than every gyro sample, is what let
    the filter finally beat raw gyro-only on demo_6.
    """

    t_gyr = rec["t_gyr"]
    yaw_rate = rec["yaw_rate"]
    dt = rec["dt"]
    heading_mag_interp = rec["heading_mag_interp"]

    step_update_indices = set(np.searchsorted(t_gyr, rec["step_times"]))

    theta = heading_mag_interp[0]
    P = R
    theta_history = [theta]

    for i in range(1, len(t_gyr)):
        theta_pred = theta + yaw_rate[i] * dt[i]
        P_pred = P + Q

        if i in step_update_indices:
            K = P_pred / (P_pred + R)
            theta = theta_pred + K * (heading_mag_interp[i] - theta_pred)
            P = (1 - K) * P_pred
        else:
            theta = theta_pred
            P = P_pred

        theta_history.append(theta)

    return np.array(theta_history)


def build_path(step_times, heading_array, heading_time_array):
    step_headings = np.interp(step_times, heading_time_array, heading_array)
    x, y = [0.0], [0.0]
    for i in range(len(step_headings)):
        x.append(x[i] + STEP_LENGTH_M * np.cos(step_headings[i]))
        y.append(y[i] + STEP_LENGTH_M * np.sin(step_headings[i]))
    return np.array(x), np.array(y)


def closing_distance(x, y):
    return np.hypot(x[-1], y[-1])


R_MULTIPLIERS_TO_TRY = [1, 10, 50, 100, 500, 2000]

print(f"{'recording':<10} {'raw (m)':>10} " + "".join(f"R x{m:<6}".rjust(10) for m in R_MULTIPLIERS_TO_TRY))

for name, windows in RECORDINGS.items():
    rec = load_recording(name)
    Q, R_base = estimate_noise(rec, windows["stationary_window_s"], windows["straight_window_s"])

    x_raw, y_raw = build_path(rec["step_times"], rec["heading_gyro_rad"], rec["t_gyr"])
    raw_cd = closing_distance(x_raw, y_raw)

    row = f"{name:<10} {raw_cd:>10.3f} "
    for mult in R_MULTIPLIERS_TO_TRY:
        heading_kf = run_kalman_periodic(rec, Q, R_base * mult)
        x_kf, y_kf = build_path(rec["step_times"], heading_kf, rec["t_gyr"])
        cd = closing_distance(x_kf, y_kf)
        marker = "*" if cd < raw_cd else " "
        row += f"{cd:>9.3f}{marker}"
    print(row)

print("\n* = beats the raw gyro-only baseline for that recording")
