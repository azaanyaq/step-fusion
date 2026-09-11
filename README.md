# StepFusion

![StepFusion result](thumbnail.png)

Pedestrian Dead Reckoning from real phone IMU data (Sensor Logger iOS app),
fused with a Kalman filter to reduce heading drift.
## File structure

- **`pdr_pipeline.py`** - the final, reusable pipeline
- **`main.py`** - runs that pipeline on every recording and plots raw
  vs. Kalman-fused paths side by side (see Results below).
- **`raw_code/`** - every exploratory script that led to the final design:
  hand-rolled vs. scipy step detection, raw heading integration, the
  magnetometer sign/unwrap debugging, the first (underperforming) Kalman
  filter, the R-value sweeps that diagnosed why, and the per-step-update
  redesign that fixed it. Kept for reference.
- **`data/raw/`** - the recorded walks (see table below).
- **`requirements.txt`**, **`.venv/`** - numpy, pandas, scipy, matplotlib.

## Data format (Sensor Logger CSV)

Columns are **`time, seconds_elapsed, z, y, x`** - note z/y/x order, not
x/y/z. Index by column name, not position. Accelerometer is m/s² with
gravity already removed (iOS); gyroscope is rad/s; magnetometer is µT.

## Experiments

| recording | environment | duration | steps | notes |
|---|---|---|---|---|
| `demo_3` | indoor | ~14.2s | ~18 | accel + gyro only, no magnetometer |
| `demo_4` | indoor | ~12.6s | 17 | first magnetometer recording |
| `demo_5` | outdoor | ~17.4s | 25 | larger rectangle |
| `demo_6` | outdoor | ~109.7s | 156 | 5 continuous loops; only recording with a counted, real step-count ground truth |

Each recording is a walk around a taped/marked rectangle, phone held flat.
`demo_3`/`demo_4` are the same small indoor rectangle; `demo_5`/`demo_6` are
a larger outdoor rectangle, recorded specifically to test whether distance
and environment change the results.

## Findings and the adjustments they led to

**Step detection**: an absolute height threshold missed weaker steps at the
start/end of a walk. Switched to `find_peaks`'s `prominence` instead, which
fixed it - `demo_6` validates this against a real count: 156 detected vs.
150 actually walked (~4% error).

**Raw gyro heading** closes to within a few degrees of 360° on every walk,
but the *position* path still drifts (1.4-3.6m), since small heading error
compounds through `cos`/`sin` at every step.

**Magnetometer fixes**: `arctan2(y, x)` needed negating to match the gyro's
rotation direction, and `np.unwrap` to remove a fake jump at the ±180°
boundary.

**First Kalman design failed on every recording**: updating from the
magnetometer at every gyro sample (~100Hz) made the fused heading track it
almost directly (~0.5s effective time constant) instead of blending.
Sweeping `R` from x1-x2000 never beat raw gyro-only, even on `demo_6`.

**Why**: `demo_4`'s magnetometer field magnitude swung ~8% (43.7-47.7 µT),
smoothly correlated with position in the room - real indoor magnetic
distortion. `demo_5` outdoors confirmed it: field far more stable (std
0.59 µT), and the filter's disadvantage shrank substantially.

**The fix**: update only at step events (~70x less often) instead of every
sample, plus retuning `R` upward - lets each rare correction carry real
weight instead of being diluted by continuous noise. This is the design in
`pdr_pipeline.py` today.

## Results

![Raw vs. Kalman-fused dead-reckoning paths for all three recordings](results.png)

| recording | steps | raw closing distance | Kalman-fused | improvement |
|---|---|---|---|---|
| `demo_4` | 17 | 1.386 m | 1.559 m | -12.6% |
| `demo_5` | 25 | 1.458 m | 1.470 m | -0.8% |
| `demo_6` | 156 | 3.637 m | 3.175 m | **+12.7%** |

*Closing distance = straight-line distance from the path's end back to its
start; the walk physically returns to its starting point, so lower is
better.*

The per-step-update fix only pays off on the long walk. `demo_4` (17
steps) and `demo_5` (25 steps) still don't beat raw gyro-only, no matter
how `R` is tuned - there aren't enough correction events for the filter's
small, systematic per-step correction to outweigh the noise each one adds.
`demo_6` (156 steps) does, by a clear and reproducible margin. 


**Magnetometer fusion helped, but only past a duration/step-count
threshold**.

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 main.py
```
