"""nav2 planning inside cartographer's LIVE map -- no map_server, no AMCL.

Why this file exists instead of `limo_bringup/limo_nav2.launch.py`:

AgileX's launch includes nav2_bringup's ``bringup_launch.py``, which pulls in
BOTH ``localization_launch.py`` (map_server + amcl) and ``navigation_launch.py``.
That is the right shape when you have a pre-built map of a room you have
already surveyed. It is the wrong shape for the exam, which is in a classroom
we have never seen: it forces the AMCL initial-pose ritual (load map -> set
2D Pose Estimate -> rotate until the scan snaps) in front of examiners, and
that ritual is the single most likely thing to fail in an unfamiliar room.

Cartographer already publishes map->odom itself, continuously, from the map it
is building as we drive. That makes AMCL redundant -- two things publishing
map->odom would in fact fight each other -- and makes map_server pointless,
since the map does not come from a file.

So: include ``navigation_launch.py`` directly and nothing else.

That choice also disposes of the lifecycle-manager trap for free. nav2's
lifecycle manager waits for every node in its ``node_names`` list to appear
and will hang forever, silently, if one never does. Rather than launching the
nodes ourselves and curating that list by hand, we reuse nav2's own
``navigation_launch.py``, whose list is already exactly:

    controller_server, planner_server, recoveries_server,
    bt_navigator, waypoint_follower

with no map_server and no amcl in it. Nothing to remove, nothing to forget.

The one thing that genuinely has to match is the /map QoS. nav2's global
costmap static layer subscribes with transient_local durability by default;
if the publisher's durability differs the costmap receives nothing and the
global planner fails with no useful error. ``map_subscribe_transient_local``
below must therefore agree with what cartographer actually publishes -- verify
with ``ros2 topic info /map --verbose`` rather than trusting this comment,
because it is a property of the cartographer build on the robot, not of us.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("limo_project_bringup")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(bringup_dir, "config", "nav2_live_slam.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    map_subscribe_transient_local = LaunchConfiguration(
        "map_subscribe_transient_local")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use the /clock topic. False on real hardware."),

        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Have the lifecycle manager configure and activate "
                        "the nav2 nodes on startup."),

        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="nav2 parameters. Defaults to our copy of AgileX's "
                        "tuned nav2.yaml -- see config/nav2_live_slam.yaml."),

        DeclareLaunchArgument(
            "map_subscribe_transient_local", default_value="true",
            description="Durability the costmap static layers use for /map. "
                        "Must match cartographer's publisher or the global "
                        "costmap silently stays empty."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, "launch",
                             "navigation_launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
                "map_subscribe_transient_local": map_subscribe_transient_local,
            }.items(),
        ),
    ])
