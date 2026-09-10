import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

DATA_DIR = "/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_4"

### Gyroscope heading (same integration approach as heading.py) ###

df_gyr = pd.read_csv(f"{DATA_DIR}/Gyroscope.csv")
t_gyr = df_gyr["seconds_elapsed"].to_numpy()
yaw_rate = df_gyr["z"].to_numpy()  # rad/s

dt = np.diff(t_gyr, prepend=t_gyr[0])
heading_gyro_rad = np.cumsum(yaw_rate * dt)

### Magnetometer heading ###

df_mag = pd.read_csv(f"{DATA_DIR}/Magnetometer.csv")
t_mag = df_mag["seconds_elapsed"].to_numpy()
mag_x = df_mag["x"].to_numpy()
mag_y = df_mag["y"].to_numpy()

# Negative so it lines up with gyro data
heading_mag_rad = -np.arctan2(mag_y, mag_x)

# Stitches the curve back to one continous line (arctan returns -180->180 only)
heading_mag_rad = np.unwrap(heading_mag_rad)

### Compare ###

heading_gyro_deg = np.degrees(heading_gyro_rad)
heading_mag_deg = np.degrees(heading_mag_rad)


# Shifting the magnetometer heading so both start at the same value (magnometer based off of NESW)
heading_mag_aligned = heading_mag_deg - heading_mag_deg[0] + heading_gyro_deg[0]

plt.plot(t_gyr, heading_gyro_deg, label="gyro (integrated)")
plt.plot(t_mag, heading_mag_aligned, label="magnetometer (aligned to same start)", alpha=0.7)
plt.title("Heading comparison: gyro integration vs. magnetometer")
plt.xlabel("Time elapsed (s)")
plt.ylabel("Heading (degrees)")
plt.legend()
plt.grid(True)

plt.show()
