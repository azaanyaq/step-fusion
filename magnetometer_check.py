import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

DATA_DIR = "/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_4"

df_mag = pd.read_csv(f"{DATA_DIR}/Magnetometer.csv")
t_mag = df_mag["seconds_elapsed"].to_numpy()
mag_x = df_mag["x"].to_numpy()
mag_y = df_mag["y"].to_numpy()
mag_z = df_mag["z"].to_numpy()

field_magnitude = np.sqrt(mag_x**2 + mag_y**2 + mag_z**2)  # uT

plt.plot(t_mag, field_magnitude)
plt.axhline(np.mean(field_magnitude), color="gray", linestyle="--", label="mean")
plt.title("Magnetometer field magnitude over time")
plt.xlabel("Time elapsed (s)")
plt.ylabel("Field magnitude (uT)")
plt.legend()
plt.grid(True)

plt.show()
