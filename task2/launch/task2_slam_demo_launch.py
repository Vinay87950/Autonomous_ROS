from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess

from launch_ros.actions import Node

###############################
# Partially copied from https://github.com/turtlebot/turtlebot4_desktop/blob/humble/turtlebot4_viz/launch/view_robot.launch.py
# for getting the Turtlebot4 Model into RViz

pkg_turtlebot4_viz = get_package_share_directory('turtlebot4_viz')
rviz2_config = PathJoinSubstitution([pkg_turtlebot4_viz, 'rviz', 'robot.rviz'])


def generate_launch_description():

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz2_config],
    )

###############################


###############################
# Used for testing with SLAM
    slam = ExecuteProcess(
        cmd=["ros2", "launch", "turtlebot4_navigation", "slam.launch.py"]
    )
###############################

    return LaunchDescription([
        slam,
        rviz2_node,
])