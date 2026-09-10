import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

DATA_DIR = "/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_4"

### Gyroscope ###

df_gyr = pd.read_csv(f"{DATA_DIR}/Gyroscope.csv")
t_gyr = df_gyr["seconds_elapsed"].to_numpy()
yaw_rate = df_gyr["z"].to_numpy()  # rad/s

dt = np.diff(t_gyr, prepend=t_gyr[0])
mean_dt = np.mean(dt[1:])  # dt[0] is the artificial 0 we inserted, skip it

heading_gyro_rad = np.cumsum(yaw_rate * dt)  # raw, uncorrected - kept only for comparison

### Magnetometer ###

df_mag = pd.read_csv(f"{DATA_DIR}/Magnetometer.csv")
t_mag = df_mag["seconds_elapsed"].to_numpy()
mag_x = df_mag["x"].to_numpy()
mag_y = df_mag["y"].to_numpy()
heading_mag_rad = np.unwrap(-np.arctan2(mag_y, mag_x))

### Noise estimates (same windows/method as noise_estimation.py) ###

STRAIGHT_WINDOW_S = (7.0, 9.3) # Flat plateau in the heading, not turning
straight_mask = (t_mag >= STRAIGHT_WINDOW_S[0]) & (t_mag <= STRAIGHT_WINDOW_S[1])
R = np.var(heading_mag_rad[straight_mask]) # Magnetometer measurement noise ( rad^2 )

STATIONARY_WINDOW_S = (0.0, 3.5) # Phone not moving - before the walk starts
stationary_mask = t_gyr <= STATIONARY_WINDOW_S[1]
Q = np.var(yaw_rate[stationary_mask]) * mean_dt**2 # Gyro process noise ( rad^2 per step )

### Align magnetometer heading onto the gyro's own timeline ###

# Interpolating magnetometer data to line up with gyro's values
heading_mag_interp = np.interp(t_gyr, t_mag, heading_mag_rad)

### Kalman filter: predict with gyro, update with magnetometer, once per gyro sample ###

theta = heading_mag_interp[0] # Start anchored to the real-world reference, not an arbitrary 0
P = R # Initial uncertainty: our only real info at t=0 is the first magnetometer reading

theta_history = [theta]

for i in range(1, len(t_gyr)):

    # Predict: project the last estimate forward using the gyro's rotation rate
    theta_pred = theta + yaw_rate[i] * dt[i]
    P_pred = P + Q

    # Update: blend the prediction with the magnetometer's reading at this instant, weighted by the Kalman gain
    K = P_pred / (P_pred + R)
    theta = theta_pred + K * (heading_mag_interp[i] - theta_pred)
    P = (1 - K) * P_pred

    theta_history.append(theta) # List of theta values according to the filter output

heading_kf_rad = np.array(theta_history) # Kalman filter heading

### Compare: raw gyro-only, raw magnetometer-only, and the fused Kalman estimate ###

heading_gyro_deg = np.degrees(heading_gyro_rad)

# Raw gyro heading starts at an arbitrary 0 - shift it to start at the same point as the others
heading_gyro_deg_aligned = heading_gyro_deg - heading_gyro_deg[0] + np.degrees(theta_history[0])

plt.plot(t_gyr, heading_gyro_deg_aligned, label="gyro only (raw, drifts)", alpha=0.6)
plt.plot(t_gyr, np.degrees(heading_mag_interp), label="magnetometer only (raw, noisy)", alpha=0.6)
plt.plot(t_gyr, np.degrees(heading_kf_rad), label="Kalman filter (fused)", linewidth=2, color="black")
plt.title("Heading: gyro-only vs. magnetometer-only vs. Kalman fusion")
plt.xlabel("Time elapsed (s)")
plt.ylabel("Heading (degrees)")
plt.legend()
plt.grid(True)

plt.show()
