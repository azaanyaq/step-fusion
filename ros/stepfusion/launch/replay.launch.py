"""
Replays a recorded walk into the StepFusion node, run from the repo root:
    ros2 launch stepfusion replay.launch.py recording:=demo_6

foxglove:=true starts foxglove_bridge (ws://<VM IP>:8765), record:=true records the outputs to a bag.
"""

import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

OUTPUT_TOPICS = ["/stepfusion_node/path", "/stepfusion_node/path_raw", "/stepfusion_node/heading"]


def launch_setup(context):
    recording = LaunchConfiguration("recording").perform(context)
    bag_root = os.path.abspath(LaunchConfiguration("bag_root").perform(context))
    bag_dir = os.path.join(bag_root, recording)
    use_foxglove = LaunchConfiguration("foxglove").perform(context).lower() == "true"
    record = LaunchConfiguration("record").perform(context).lower() == "true"
    share = get_package_share_directory("stepfusion")

    if not os.path.isdir(bag_dir):
        raise RuntimeError(f"no bag at {bag_dir} - run `python tools/csv_to_bag.py {recording}` from the repo root")

    actions = [
        Node(
            package="stepfusion",
            executable="stepfusion_node",
            output="screen",
            parameters=[os.path.join(share, "config", f"{recording}.yaml"), {"use_sim_time": True}],
        ),
    ]

    if use_foxglove:
        actions.append(Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            parameters=[{"port": 8765, "use_sim_time": True}],
        ))

    if record:
        output_dir = os.path.join(bag_root, f"{recording}_output_{time.strftime('%Y%m%d_%H%M%S')}")
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "--use-sim-time", "-o", output_dir] + OUTPUT_TOPICS,
            output="screen",
        ))

    # Delay so everything has subscribed before the bag starts
    actions.append(TimerAction(
        period=3.0,
        actions=[ExecuteProcess(cmd=["ros2", "bag", "play", bag_dir, "--clock"], output="screen")],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("recording", default_value="demo_6", description="demo_4, demo_5 or demo_6"),
        DeclareLaunchArgument("bag_root", default_value="data/bags", description="folder holding the converted bags"),
        DeclareLaunchArgument("foxglove", default_value="true", description="start foxglove_bridge on port 8765"),
        DeclareLaunchArgument("record", default_value="false", description="record the node's outputs to a bag"),
        OpaqueFunction(function=launch_setup),
    ])
