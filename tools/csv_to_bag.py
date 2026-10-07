"""
Convert a Sensor Logger recording (Accelerometer/Gyroscope/Magnetometer CSVs)
into a ROS 2 bag that `ros2 bag play` can replay into the StepFusion node.

Uses the pure-Python `rosbags` library, so it runs with or without ROS
installed (pip install rosbags).

Run from the repo root:
    python tools/csv_to_bag.py demo_6            -> data/bags/demo_6/
    python tools/csv_to_bag.py demo_4 demo_5 demo_6

Topics written:
    /imu/data_raw  sensor_msgs/Imu            gyro (rad/s) + accel (m/s^2)
    /imu/mag       sensor_msgs/MagneticField  field in Tesla (CSV is uT)

Decisions baked in here - each one is yours to defend, revisit in session 7:

  - Topic names follow the imu_tools convention (data_raw = no orientation
    estimate yet, mag alongside it).
  - Data is left in the phone's own axes, frame_id "phone_imu". Mapping that
    to base_link (REP-103) is a static transform in the launch file, not
    something this script fakes by rotating numbers.
  - Accelerometer is written AS RECORDED, i.e. with gravity already removed
    by iOS. REP-145 says Imu.linear_acceleration should include gravity, so
    these messages knowingly break that convention.
  - orientation_covariance[0] = -1 (message docs: "no orientation estimate").
    Other covariances are all zero, which the docs define as "unknown".
  - Header stamps and bag timestamps are the CSV's `time` column (epoch ns),
    i.e. when the phone sampled, not when this script ran.
  - At equal timestamps the mag message is written before the Imu message,
    so a node that processes in arrival order sees the same-instant mag
    reading when a step fires (matching the Python reference). ROS itself
    does not guarantee ordering across topics, so the node must not rely on it.
"""

import os
import sys

import numpy as np
import pandas as pd
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

FRAME_ID = "phone_imu"
IMU_TOPIC = "/imu/data_raw"
MAG_TOPIC = "/imu/mag"
UT_TO_T = 1e-6

typestore = get_typestore(Stores.ROS2_JAZZY)
Imu = typestore.types["sensor_msgs/msg/Imu"]
MagneticField = typestore.types["sensor_msgs/msg/MagneticField"]
Header = typestore.types["std_msgs/msg/Header"]
Time = typestore.types["builtin_interfaces/msg/Time"]
Vector3 = typestore.types["geometry_msgs/msg/Vector3"]
Quaternion = typestore.types["geometry_msgs/msg/Quaternion"]


def header(t_ns):
    return Header(stamp=Time(sec=int(t_ns // 1_000_000_000), nanosec=int(t_ns % 1_000_000_000)), frame_id=FRAME_ID)


def convert(name):
    data_dir = f"data/raw/{name}"
    out_dir = f"data/bags/{name}"
    if os.path.exists(out_dir):
        raise SystemExit(f"{out_dir} already exists - delete it first if you want to regenerate it")

    acc = pd.read_csv(f"{data_dir}/Accelerometer.csv")
    gyr = pd.read_csv(f"{data_dir}/Gyroscope.csv")
    mag = pd.read_csv(f"{data_dir}/Magnetometer.csv")

    # One Imu message carries gyro + accel together, so they must be sampled together
    if not np.array_equal(acc["time"].to_numpy(), gyr["time"].to_numpy()):
        raise SystemExit(f"{name}: accelerometer and gyroscope timestamps differ - can't pair them into Imu messages")

    no_orientation = np.zeros(9)
    no_orientation[0] = -1.0
    unknown = np.zeros(9)

    # (timestamp_ns, write_order, topic, serialized message); write_order puts mag first at equal times
    events = []

    for t_ns, mx, my, mz in mag[["time", "x", "y", "z"]].itertuples(index=False):
        msg = MagneticField(
            header=header(t_ns),
            magnetic_field=Vector3(x=mx * UT_TO_T, y=my * UT_TO_T, z=mz * UT_TO_T),
            magnetic_field_covariance=unknown,
        )
        events.append((int(t_ns), 0, MAG_TOPIC, typestore.serialize_cdr(msg, MagneticField.__msgtype__)))

    acc_xyz = acc[["x", "y", "z"]].to_numpy()
    for i, (t_ns, gx, gy, gz) in enumerate(gyr[["time", "x", "y", "z"]].itertuples(index=False)):
        ax, ay, az = acc_xyz[i]
        msg = Imu(
            header=header(t_ns),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            orientation_covariance=no_orientation,
            angular_velocity=Vector3(x=gx, y=gy, z=gz),
            angular_velocity_covariance=unknown,
            linear_acceleration=Vector3(x=ax, y=ay, z=az),
            linear_acceleration_covariance=unknown,
        )
        events.append((int(t_ns), 1, IMU_TOPIC, typestore.serialize_cdr(msg, Imu.__msgtype__)))

    events.sort(key=lambda e: (e[0], e[1]))

    os.makedirs(os.path.dirname(out_dir), exist_ok=True)
    with Writer(out_dir, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        connections = {
            IMU_TOPIC: writer.add_connection(IMU_TOPIC, Imu.__msgtype__, typestore=typestore),
            MAG_TOPIC: writer.add_connection(MAG_TOPIC, MagneticField.__msgtype__, typestore=typestore),
        }
        for t_ns, _, topic, raw in events:
            writer.write(connections[topic], t_ns, raw)

    duration_s = (events[-1][0] - events[0][0]) / 1e9
    print(f"{name}: {len(gyr)} Imu + {len(mag)} MagneticField messages, {duration_s:.1f} s -> {out_dir}/")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit("usage: python tools/csv_to_bag.py <recording> [<recording> ...]")
    for name in sys.argv[1:]:
        convert(name)
