"""nav2 localising against the SAVED classroom map with AMCL.

Counterpart to nav2_live_slam.launch.py, which plans inside cartographer's
live map. That file exists for an unsurveyed room; this one exists now that
the room IS surveyed -- examroom_t45 was driven and saved on 27 Aug 2026 and
its coordinate skeleton is in config/room_skeleton.yaml.

CARTOGRAPHER MUST BE STOPPED before this runs. Both cartographer and AMCL
publish map->odom. Two publishers on one transform do not fall back to each
other; the TF tree flickers between them and every pose in the stack becomes
unreliable in a way that reads as random nav2 failure.

Default map is examroom_t45, not examroom. Same drive, different occupied
threshold: the classroom table and chair legs sit at 45-64 in cartographer's
probability grid, so at the standard 0.65 they are written out as "unknown"
and nav2 will plan straight through a table. At 0.45 they become real
obstacles -- 6008 occupied cells against 2743 -- with no change to the walls.
See scripts/save_map.py for why map_saver_cli could not do this.

The initial pose is start_pose from room_skeleton.yaml, which is the origin by
construction: cartographer was restarted with the robot already standing on
the entrance spot, so (0, 0, 0) IS that spot. Publish it to /initialpose, or
let the mission node's LOCALIZE phase do it.
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
    default_map = os.path.join(bringup_dir, "maps", "examroom_t45.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")

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
            description="nav2 parameters. The same copy of AgileX's tuned "
                        "nav2.yaml -- its amcl and map_server sections were "
                        "always present and simply unused until now."),

        DeclareLaunchArgument(
            "map", default_value=default_map,
            description="Saved map yaml. examroom_t45 (0.45 threshold) by "
                        "default; pass examroom.yaml for the 0.65 version."),

        # bringup_launch.py pulls in BOTH localization_launch.py (map_server +
        # amcl) and navigation_launch.py, and its lifecycle manager node_names
        # lists are already correct for that combination. Nothing to curate by
        # hand, which is what made the live-SLAM variant hang before.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
                "slam": "False",
            }.items(),
        ),
    ])
