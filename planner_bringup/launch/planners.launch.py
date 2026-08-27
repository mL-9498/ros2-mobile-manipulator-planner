from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="unordered_task_planner",
            executable="set_cover_node",
            name="set_cover_planner",
            output="screen",
        ),
        Node(
            package="grid_motion_planner",
            executable="astar_node",
            name="grid_motion_planner",
            output="screen",
        ),
    ])
