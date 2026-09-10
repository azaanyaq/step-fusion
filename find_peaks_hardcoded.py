import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df_acc = pd.read_csv("/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_4/Accelerometer.csv")
df_gyr = pd.read_csv("/Users/azaanyaqub/Developer/kalman-filter/iphone-project/data/raw/demo_4/Gyroscope.csv")

df_acc["magnitude"] = ( df_acc["x"]**2 + df_acc["y"]**2 + df_acc["z"]**2 ) ** 0.5

# Convert pd columns to numpy arrays (easier indexing)
mag = df_acc["magnitude"].to_numpy()
t = df_acc["seconds_elapsed"].to_numpy()

# Estimating sample rate (so works for any Hz data used)
sampling_rate_hz = 1 / np.mean(np.diff(t))

# Peak detection parameters
HEIGHT_THRESHOLD = 2.0
MIN_STEP_GAP_S = 0.4
min_step_gap_samples = int(MIN_STEP_GAP_S * sampling_rate_hz)

peak_indices = []

# Done so that first peak isn't rejected if too early on in the time span
last_peak_index = -min_step_gap_samples

# Skip i=0 and i=len(mag)-1 as need points before and after for this algorithm
for i in range(1, len(mag) - 1):
    is_local_max = mag[i] > mag[i - 1] and mag[i] > mag[i + 1] # Greater than sample before and after
    is_tall_enough = mag[i] > HEIGHT_THRESHOLD # Does it exceed the threshold
    is_far_enough = (i - last_peak_index) >= min_step_gap_samples # Is the gap large enough to be its own peak

    if is_local_max and is_tall_enough and is_far_enough: 
        peak_indices.append(i) # Recording as a peak 
        last_peak_index = i # Storing index of current peak

print(f"Detected {len(peak_indices)} steps (expected roughly 18)")

plt.plot(t, mag, label="acceleration magnitude")
plt.scatter(t[peak_indices], mag[peak_indices], color="red", marker="x", s=80, label="detected step")
plt.title('Data plot')
plt.xlabel('Time elapsed (s)')
plt.ylabel('Magnitude')
plt.legend()
plt.grid(True)

plt.show()