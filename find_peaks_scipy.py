import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import find_peaks

df_acc = pd.read_csv("/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_3/Accelerometer.csv")

df_acc["magnitude"] = ( df_acc["x"]**2 + df_acc["y"]**2 + df_acc["z"]**2 ) ** 0.5

mag = df_acc["magnitude"].to_numpy()
t = df_acc["seconds_elapsed"].to_numpy()

sampling_rate_hz = 1 / np.mean(np.diff(t))

# Same starting parameters as the manual version, for a fair comparison
HEIGHT_THRESHOLD = 1.0
MIN_STEP_GAP_S = 0.4
min_step_gap_samples = int(MIN_STEP_GAP_S * sampling_rate_hz)

# Returns array of accepted indices (and a dict on extra info about each one)
peak_indices, _properties = find_peaks(mag, prominence=HEIGHT_THRESHOLD, distance=min_step_gap_samples)

print(f"Detected {len(peak_indices)} steps (expected roughly 18)")

plt.plot(t, mag, label="acceleration magnitude")
plt.scatter(t[peak_indices], mag[peak_indices], color="red", marker="x", s=80, label="detected step")
plt.title('Data plot (scipy find_peaks)')
plt.xlabel('Time elapsed (s)')
plt.ylabel('Magnitude')
plt.legend()
plt.grid(True)

plt.show()