"""
Converts a Sensor Logger recording into a ROS 2 bag (data/bags/<recording>/), run from the repo root:
    python tools/csv_to_bag.py demo_6

  - /imu/data_raw (sensor_msgs/Imu): gyro + accel, accel left with gravity removed (breaks REP-145)
  - /imu/mag (sensor_msgs/MagneticField): converted from uT to Tesla
  - Left in the phone's own axes (frame_id phone_imu), stamped with the CSV's time column
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

    # One Imu message carries both, so they must share timestamps
    if not np.array_equal(acc["time"].to_numpy(), gyr["time"].to_numpy()):
        raise SystemExit(f"{name}: accelerometer and gyroscope timestamps differ - can't pair them into Imu messages")

    no_orientation = np.zeros(9)
    no_orientation[0] = -1.0
    unknown = np.zeros(9)

    # (timestamp, order, topic, data) - mag written first on equal timestamps
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
