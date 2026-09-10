import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import find_peaks

df_acc = pd.read_csv("/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_4/Accelerometer.csv")
df_gyr = pd.read_csv("/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_4/Gyroscope.csv")

### Code from find_peaks_scipy ###

df_acc["magnitude"] = ( df_acc["x"]**2 + df_acc["y"]**2 + df_acc["z"]**2 ) ** 0.5

mag = df_acc["magnitude"].to_numpy()
t_acc = df_acc["seconds_elapsed"].to_numpy()

sampling_rate_hz = 1 / np.mean(np.diff(t_acc))

# Same starting parameters as the manual version, for a fair comparison
HEIGHT_THRESHOLD = 1.0
MIN_STEP_GAP_S = 0.4
min_step_gap_samples = int(MIN_STEP_GAP_S * sampling_rate_hz)

# Returns array of accepted indices (and a dict on extra info about each one)
peak_indices, _properties = find_peaks(mag, prominence=HEIGHT_THRESHOLD, distance=min_step_gap_samples)

### Code from heading ###

t_gyr = df_gyr["seconds_elapsed"].to_numpy()
yaw_rate = df_gyr["z"].to_numpy() # Rotation rate around the phone's vertical axis (in rad/s)

# Time gap between each sample and the prior one
dt = np.diff(t_gyr, prepend=t_gyr[0]) # prepend inserts a virtual point = t[0] first so dt[0] = 0

# Discrete integration ( sigma(w*dt) ) to get the angle in rads
heading_rad = np.cumsum(yaw_rate * dt)

### Interpolation code ###

# Set parameter
STEP_LENGTH_M = 0.73

step_times = t_acc[peak_indices] # seconds, from your step detector's timeline
step_headings = np.interp(step_times, t_gyr, heading_rad) # heading at each step time

x, y = [0.0], [0.0]

# The loop
for i in range(len(step_headings)):
    x_current = x[i] + STEP_LENGTH_M * np.cos(step_headings[i])
    y_current = y[i] + STEP_LENGTH_M * np.sin(step_headings[i])
    x.append(x_current)
    y.append(y_current)

plt.plot(x, y, marker='o')
plt.title("Raw dead-reckoning path")
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.gca().set_aspect('equal')
plt.grid(True)

plt.show()