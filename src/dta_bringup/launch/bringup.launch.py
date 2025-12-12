#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package share dirs
    warehouse_world_dir = get_package_share_directory('warehouse_world')
    localization_dir = get_package_share_directory('localization')
    motion_planner_dir = get_package_share_directory('motion_planner')

    # 1) ros2 launch warehouse_world spawn_five_tb3.launch.py
    spawn_tb3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(warehouse_world_dir, 'launch', 'spawn_five_tb3.launch.py')
        )
    )

    # 2) ros2 launch localization multi_amcl.launch.py
    multi_amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_dir, 'launch', 'multi_amcl.launch.py')
        )
    )

    # 3) ros2 run path_planning inflation_costmap
    inflation_costmap_node = Node(
        package='path_planning',
        executable='inflation_costmap',
        name='inflation_costmap',
        output='screen'
    )

    # 4) ros2 run path_planning a_star_planner
    a_star_planner_node = Node(
        package='path_planning',
        executable='a_star_planner',
        name='a_star_planner',
        output='screen'
    )

    # 5) ros2 launch motion_planner pd_motion_planner_multi.launch.py
    pd_multi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(motion_planner_dir, 'launch', 'pd_motion_planner_multi.launch.py')
        )
    )

    # 6) rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        # If you have an RViz config:
        # arguments=['-d', '/home/mudit/turtlebot3_ws/src/your_pkg/config/your_config.rviz']
    )

    # 7) ros2 run path_planning path_allocation
    path_allocation_node = Node(
        package='path_planning',
        executable='path_allocation',
        name='path_allocation',
        output='screen'
    )

    return LaunchDescription([
        # Start Gazebo + 5 TB3 immediately
        spawn_tb3_launch,

        # Start multi_amcl after 5 seconds
        TimerAction(
            period=5.0,
            actions=[multi_amcl_launch]
        ),

        # Start inflation_costmap after 10 seconds
        TimerAction(
            period=10.0,
            actions=[inflation_costmap_node]
        ),

        # Start a_star_planner after 15 seconds
        TimerAction(
            period=15.0,
            actions=[a_star_planner_node]
        ),

        # Start multi PD motion planners after 20 seconds
        TimerAction(
            period=20.0,
            actions=[pd_multi_launch]
        ),

        # Start RViz after 25 seconds
        TimerAction(
            period=25.0,
            actions=[rviz_node]
        ),

        # Start path_allocation after 30 seconds
        TimerAction(
            period=30.0,
            actions=[path_allocation_node]
        ),
    ])
