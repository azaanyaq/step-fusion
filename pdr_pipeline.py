"""
Pedestrian Dead Reckoning + Kalman filter pipeline.

Turns raw phone IMU recordings (Accelerometer, Gyroscope, Magnetometer) into a 
2D walked path, two ways:

  - Raw: step detection + gyro-integrated heading only (drifts over time)

  - Fused: same but heading is corrected with a Kalman filter that
    predicts from the gyro every sample and updates from the magnetometer
    only at detected step events
"""

import numpy as np
import pandas as pd
from scipy.signal import find_peaks

STEP_LENGTH_M = 0.73 # Fixed literature typical average adult step length (didn't measure my own)


def load_recording(data_dir):

    # Load one recording's three sensor CSVs into arrays keyed by sensor
    df_acc = pd.read_csv(f"{data_dir}/Accelerometer.csv")
    df_gyr = pd.read_csv(f"{data_dir}/Gyroscope.csv")
    df_mag = pd.read_csv(f"{data_dir}/Magnetometer.csv")

    # Store relevant pd columns into a dictionary for easy access, converting to np arrays for indexing
    return {
        "t_acc": df_acc["seconds_elapsed"].to_numpy(),
        "acc_x": df_acc["x"].to_numpy(),
        "acc_y": df_acc["y"].to_numpy(),
        "acc_z": df_acc["z"].to_numpy(),
        "t_gyr": df_gyr["seconds_elapsed"].to_numpy(),
        "yaw_rate": df_gyr["z"].to_numpy(),  # rad/s
        "t_mag": df_mag["seconds_elapsed"].to_numpy(),
        "mag_x": df_mag["x"].to_numpy(),
        "mag_y": df_mag["y"].to_numpy(),
    }


def detect_steps(rec):

    # Detects steps from accelerometer magnitude spikes
    magnitude = np.sqrt(rec["acc_x"] ** 2 + rec["acc_y"] ** 2 + rec["acc_z"] ** 2)
    t_acc = rec["t_acc"]

    sampling_rate_hz = 1 / np.mean(np.diff(t_acc))
    min_step_gap_samples = int(0.4 * sampling_rate_hz) # Steps can't happen faster than ~0.4s apart

    peak_indices, _ = find_peaks(magnitude, prominence=1.0, distance=min_step_gap_samples)
    return t_acc[peak_indices] # Returns indexes of steps


def compute_gyro_heading(rec):

    # Computes raw heading from integrated gyro yaw rate
    t_gyr = rec["t_gyr"]
    dt = np.diff(t_gyr, prepend=t_gyr[0])

    return np.cumsum(rec["yaw_rate"] * dt) # Numerical integration


def compute_magnetometer_heading(rec):
   
    # Heading from magnetometers horizontal field vector
    heading = -np.arctan2(rec["mag_y"], rec["mag_x"]) # -ive to line up with raw heading
    return np.unwrap(heading) # Unwrapped to return one continous curve (rather than jumps)


def estimate_noise(rec, stationary_window_s, straight_window_s):
    
    # Estimate Q and R from real segements of recording
    t_gyr = rec["t_gyr"]
    dt = np.diff(t_gyr, prepend=t_gyr[0])
    mean_dt = np.mean(dt[1:])

    """
    Window boundaries picked by eye per recoding (flat regions). Two windows:
      
      - Stationary (for Q): flat region at the start to see if yaw rate has jitter
      - Straight (for R): plateau in middle of the walk (walking in a straight line)
    """

    stationary_mask = t_gyr <= stationary_window_s[1] 
    Q = np.var(rec["yaw_rate"][stationary_mask]) * mean_dt**2

    heading_mag = compute_magnetometer_heading(rec)
    t_mag = rec["t_mag"]
    straight_mask = (t_mag >= straight_window_s[0]) & (t_mag <= straight_window_s[1])
    R = np.var(heading_mag[straight_mask])

    return Q, R


def run_kalman_filter(rec, step_times, Q, R):

    # Fuse gyro + magnetometer into a correct heading estimate
    
    t_gyr = rec["t_gyr"]
    yaw_rate = rec["yaw_rate"]
    dt = np.diff(t_gyr, prepend=t_gyr[0])

    heading_mag = compute_magnetometer_heading(rec)
    heading_mag_interp = np.interp(t_gyr, rec["t_mag"], heading_mag)

    step_update_indices = set(np.searchsorted(t_gyr, step_times))

    theta = heading_mag_interp[0] # Anchor to real world reference

    """
    Q: process noise, a single fixed number representing how much uncertainty gets added to the heading 
       estimate on each single prediction step, due to gyro noise

    R: measurement noise, a single fixed number representing how noisy each magetometer reading is 
    
    P: filter's current uncertainty about its own heading estimate, changes every iteration
    
    P_pred: P after one predict step (P_pred = P + Q)
    
    K: kalman gain, value from 0 -> 1 computed fresh only during updates, represents the trust in the new 
       magnetometer reading against the prediction
    """

    P = R
    theta_history = [theta]

    for i in range(1, len(t_gyr)): # Predict for all samples (~11,000 for demo_6)
        theta_pred = theta + yaw_rate[i] * dt[i]
        P_pred = P + Q

        if i in step_update_indices: # Update only when a step occurs (to avoid magnetometer noise worsening prediction)
            K = P_pred / (P_pred + R)
            theta = theta_pred + K * (heading_mag_interp[i] - theta_pred)
            P = (1 - K) * P_pred
        else:
            theta = theta_pred
            P = P_pred

        theta_history.append(theta)

    return np.array(theta_history)


def build_path(step_times, heading_array, heading_time_array):

    # Dead reckon a 2D path - one step froward in the current heading's direction, per detected step

    step_headings = np.interp(step_times, heading_time_array, heading_array) # Aligning data with common time steps

    x, y = [0.0], [0.0]
    for i in range(len(step_headings)):
        x.append(x[i] + STEP_LENGTH_M * np.cos(step_headings[i]))
        y.append(y[i] + STEP_LENGTH_M * np.sin(step_headings[i]))

    return np.array(x), np.array(y)


def closing_distance(x, y):

    # Straight line distance from path's start -> end point (should be 0m)
    return np.hypot(x[-1], y[-1])
