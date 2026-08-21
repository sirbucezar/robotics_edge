#!/usr/bin/env python3
"""One launch file for the whole project layer.

    ros2 launch limo_project_bringup mission.launch.py detector:=mock
    ros2 launch limo_project_bringup mission.launch.py detector:=yolo

Deliberately does NOT start ``limo_start.launch.py`` or nav2 -- bring those up
separately. Colour camera is owned by this launch (``color_camera_node``,
behind ``use_color_node``). Never launch ``orbbec_camera`` / ``dabai_d1.launch.py``
-- see docs/00_limo_pro_reference.md under Gotchas.

Expected order in three terminals:

    1. ros2 launch limo_bringup limo_start.launch.py
    2. ros2 launch limo_bringup limo_nav2.launch.py
    3. ros2 launch limo_project_bringup mission.launch.py detector:=yolo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("limo_project_bringup")
    default_params = os.path.join(pkg, "config", "mission_params.yaml")

    detector = LaunchConfiguration("detector")
    params_file = LaunchConfiguration("params_file")
    use_dashboard = LaunchConfiguration("use_dashboard")
    use_mission = LaunchConfiguration("use_mission")
    use_color_node = LaunchConfiguration("use_color_node")
    log_level = LaunchConfiguration("log_level")

    args = [
        DeclareLaunchArgument(
            "detector", default_value="mock",
            description="mock | yolo | none. 'mock' needs no model and no camera."),
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="Full path to the parameter yaml."),
        DeclareLaunchArgument("use_dashboard", default_value="true"),
        DeclareLaunchArgument("use_mission", default_value="true"),
        DeclareLaunchArgument(
            "use_color_node", default_value="true",
            description="Publish /camera/color/image_raw ourselves via V4L2. "
                        "Turn off only if something else is already publishing it."),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    common = dict(
        parameters=[params_file],
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level],
    )

    mock_detector = Node(
        package="limo_perception",
        executable="mock_detector_node",
        name="mock_detector",
        condition=IfCondition(PythonExpression(["'", detector, "' == 'mock'"])),
        **common,
    )

    yolo_detector = Node(
        package="limo_perception",
        executable="yolo_detector_node",
        name="yolo_detector",
        condition=IfCondition(PythonExpression(["'", detector, "' == 'yolo'"])),
        **common,
    )

    color_camera = Node(
        package="limo_perception",
        executable="color_camera_node",
        name="color_camera",
        condition=IfCondition(use_color_node),
        **common,
    )

    tracker = Node(
        package="limo_people",
        executable="people_tracker_node",
        name="people_tracker",
        **common,
    )

    mission = Node(
        package="limo_mission",
        executable="mission_node",
        name="mission",
        condition=IfCondition(use_mission),
        **common,
    )

    dashboard = Node(
        package="limo_dashboard",
        executable="dashboard_node",
        name="dashboard",
        condition=IfCondition(use_dashboard),
        **common,
    )

    return LaunchDescription(args + [color_camera, mock_detector, yolo_detector,
                                     tracker, mission, dashboard])
