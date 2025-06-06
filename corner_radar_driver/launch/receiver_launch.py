# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_yaml(path):
    with open(path, "r") as file:
        import yaml

        return yaml.safe_load(file)


def generate_launch_description():
    """
    Generate a ROS 2 launch description for the corner_radar_driver receiver node.

    Generate a launch description containing the receiver node and associated static
    transform publishers for all active sensors.

    Returns
    -------
    LaunchDescription
        A ROS 2 launch description object containing all necessary nodes
        for the receiver and static transforms.

    """
    pkg_dir = get_package_share_directory("corner_radar_driver")

    receiver_params = os.path.join(pkg_dir, "config", "receiver_params.yaml")

    sensors_config_file = os.path.join(pkg_dir, "config", "sensors_configuration.yaml")
    sensors_full_config = load_yaml(sensors_config_file)
    sensors_config = sensors_full_config["corner_radar_driver_receiver"][
        "ros__parameters"
    ]["sensors"]

    static_tf_nodes = []
    for sensor_name, sensor in sensors_config.items():
        # Create a static transform publisher for each sensor
        if sensor.get("active", True):
            pos = sensor["mounting_position"]
            static_tf_nodes.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher_{sensor_name}",
                    arguments=[
                        str(float(pos["xt"])),
                        str(float(pos["yt"])),
                        str(float(pos["zt"])),
                        str(float(pos["yaw"])),
                        str(float(pos["pitch"])),
                        str(float(pos["roll"])),
                        "base_link",
                        sensor_name,
                    ],
                    output="screen",
                )
            )

    # Create the receiver node
    receiver_node = Node(
        package="corner_radar_driver",
        executable="receiver",
        name="corner_radar_driver_receiver",
        parameters=[sensors_config_file, receiver_params],
        output="screen",
    )
    return LaunchDescription(static_tf_nodes + [receiver_node])
