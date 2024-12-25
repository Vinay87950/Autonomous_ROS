from ament_index_python.packages import get_package_share_directory

import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node

###############################
# Partially copied from https://github.com/turtlebot/turtlebot4_desktop/blob/humble/turtlebot4_viz/launch/view_robot.launch.py
# for getting the Turtlebot4 Model into RViz

pkg_turtlebot4_viz = get_package_share_directory('turtlebot4_viz')
rviz2_config = PathJoinSubstitution([pkg_turtlebot4_viz, 'rviz', 'robot.rviz'])

# rviz2_config = os.path.join(os.getcwd(), "config.rviz")
# rviz2_config = PathJoinSubstitution([os.getcwd(), 'config.rviz'])

def generate_launch_description():

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/scan', 'scan'),
        ('/map', 'map'),
        ('/map_metadata', 'map_metadata'),
    ]
    
    my_package_dir = get_package_share_directory("task2")

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', os.path.join(my_package_dir, "launch" , "visualize_scan.rviz")],
        # remappings=remappings
    )

###############################

    mapping_node = Node(
        package="task2",
        executable="mapping",
        name="mapping"
    )

    return LaunchDescription([
        mapping_node,
        rviz2_node,
])