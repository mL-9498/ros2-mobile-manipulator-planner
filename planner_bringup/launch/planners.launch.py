import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("planner_bringup"),
        "rviz",
        "synthetic_demo.rviz",
    )
    return LaunchDescription([
        Node(
            package="unordered_task_planner",
            executable="set_cover_node",
            name="set_cover_planner",
            output="screen",
        ),
        Node(
            package="unordered_task_planner",
            executable="dijkstra_node",
            name="dijkstra_task_planner",
            output="screen",
        ),
        Node(
            package="grid_motion_planner",
            executable="astar_node",
            name="grid_motion_planner",
            output="screen",
        ),
        Node(
            package="planner_bringup",
            executable="synthetic_demo",
            name="synthetic_planning_demo",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
