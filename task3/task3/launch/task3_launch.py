import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    path_planner_node = Node(
        package="task3",
        executable="path_planning",
        name="path_planning"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d", os.path.join(get_package_share_directory("task3"), "launch", "config.rviz")],
    )

    slam = ExecuteProcess(
        cmd=["ros2", "launch", "turtlebot4_navigation", "slam.launch.py"])

    return LaunchDescription([
        path_planner_node,
        rviz2_node,
        slam,
    ])
