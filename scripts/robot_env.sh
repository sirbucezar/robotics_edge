# Source this in every non-interactive shell on the robot.
#
# WHY: ~/.bashrc sets ROS_DOMAIN_ID=2 and RMW_IMPLEMENTATION=rmw_cyclonedds_cpp.
# `ssh host "cmd"` and `bash -c` never read .bashrc, so anything launched that
# way lands on domain 0 with the default RMW and is invisible to the chassis
# bringup started from an interactive shell -- topics list fine and deliver
# nothing. Cost us a cartographer that saw zero scans.
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///var/lib/theconstruct.rrl/cyclonedds.xml
source /opt/ros/foxy/setup.bash
[ -f ~/limo_ros2_ws/install/setup.bash ] && source ~/limo_ros2_ws/install/setup.bash
[ -f ~/limo_project/ros2_ws/install/setup.bash ] && source ~/limo_project/ros2_ws/install/setup.bash
