# PDR + Kalman Filter (iPhone IMU)

Pedestrian Dead Reckoning from real phone IMU data (Sensor Logger iOS app),
fused with a Kalman filter to reduce drift. Built for CV/interview prep —
every part of this is meant to be understood and defensible line-by-line,
not just working.

## Data format (Sensor Logger CSV)

Columns are **`time, seconds_elapsed, z, y, x`** — note z/y/x order, not x/y/z.
Index by column name, not position, to avoid silently swapping axes.

- `time`: nanosecond epoch timestamp
- `seconds_elapsed`: seconds since recording start (use this for integration)
- Accelerometer x/y/z: m/s², gravity already removed (iOS)
- Gyroscope x/y/z: rad/s

First recorded sample (`data/raw/demo_3/`): a walk around a taped rectangle,
~18 steps, ~14.2s, sampled at ~100.4 Hz.

## Roadmap

1. Load accelerometer + gyroscope CSVs
2. Step detection — peak detection on accelerometer magnitude
3. Heading estimation — integrate gyroscope z (yaw rate) over time
4. Raw dead-reckoning position from steps + heading (drift baseline)
5. Kalman filter fusing step/heading estimate with a simple motion model
6. Validation — compare estimated path to the taped rectangle, quantify
   drift reduction from a real measurement

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```
