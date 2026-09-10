import numpy as np
import pandas as pd

DATA_DIR = "/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_4"

### Gyroscope: yaw_rate and dt, same as heading.py ###

df_gyr = pd.read_csv(f"{DATA_DIR}/Gyroscope.csv")
t_gyr = df_gyr["seconds_elapsed"].to_numpy()
yaw_rate = df_gyr["z"].to_numpy()

dt = np.diff(t_gyr, prepend=t_gyr[0])
mean_dt = np.mean(dt[1:]) # dt[0] is the artificial 0 we inserted, skip it

### Magnetometer heading, same as heading_comparison.py ###

df_mag = pd.read_csv(f"{DATA_DIR}/Magnetometer.csv")
t_mag = df_mag["seconds_elapsed"].to_numpy()
mag_x = df_mag["x"].to_numpy()
mag_y = df_mag["y"].to_numpy()
heading_mag_rad = np.unwrap(-np.arctan2(mag_y, mag_x))

### Estimate R from a straight-walking window (heading roughly constant) ###

# A flat plateau between two of the staircase's corner-drops (no change in heading)
STRAIGHT_WINDOW_S = (7.0, 9.3)
straight_mask = (t_mag >= STRAIGHT_WINDOW_S[0]) & (t_mag <= STRAIGHT_WINDOW_S[1])
R = np.var(heading_mag_rad[straight_mask])

### Estimate Q from a stationary window (phone not rotating at all) ###

# Same logic as above
STATIONARY_WINDOW_S = (0.0, 3.5)
stationary_mask = t_gyr <= STATIONARY_WINDOW_S[1]
yaw_rate_var = np.var(yaw_rate[stationary_mask]) # Noise variance of the yaw rate itself ( (rad/s)^2 )
Q = yaw_rate_var * mean_dt**2 # Converted to per-step angle-uncertainty ( rad^2 )

print(f"R (measurement noise, rad^2): {R:.6f}")
print(f"Q (process noise, rad^2 per step): {Q:.8f}")
