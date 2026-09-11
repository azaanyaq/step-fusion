import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df_gyr = pd.read_csv("/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_4/Gyroscope.csv")

t = df_gyr["seconds_elapsed"].to_numpy()
yaw_rate = df_gyr["z"].to_numpy() # Rotation rate around the phone's vertical axis (in rad/s)

# Time gap between each sample and the prior one
dt = np.diff(t, prepend=t[0]) # Prepend inserts a virtual point = t[0] first so dt[0] = 0

# Discrete integration ( sigma(w*dt) ) to get the angle in rads
heading_rad = np.cumsum(yaw_rate * dt)

# Converting rads to degrees and plotting
plt.plot(t, np.degrees(heading_rad))
plt.title("Heading over time (raw gyro integration)")
plt.xlabel("Time elapsed (s)")
plt.ylabel("Heading (degrees, relative to starting orientation)")
plt.grid(True)

plt.show()