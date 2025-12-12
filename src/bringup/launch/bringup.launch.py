import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 1) Spawn all 5 TurtleBots in the warehouse world
    spawn_five_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('warehouse_world'),
                'launch',
                'spawn_five_tb3.launch.py'
            )
        )
    )

    # 2) Start multi-robot AMCL localization
    multi_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('localization'),
                'launch',
                'multi_amcl.launch.py'
            )
        )
    )

    # 3) Inflation costmap node (path_planning)
    inflation_costmap = Node(
        package='path_planning',
        executable='inflation_costmap',
        name='inflation_costmap',
        output='screen'
    )

    # 4) RViz2 (no config file here, you can add one later if you want)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 5) A* planner node
    a_star_planner = Node(
        package='path_planning',
        executable='a_star_planner',
        name='a_star_planner',
        output='screen'
    )

    # 6) PD multi-robot motion planner launch
    pd_motion_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('motion_planner'),
                'launch',
                'pd_motion_planner_multi.launch.py'
            )
        )
    )

    # 7) Path allocation node
    path_allocation = Node(
        package='path_planning',
        executable='path_allocation',
        name='path_allocation',
        output='screen'
    )

    # Use timers to loosely respect the order you currently run things in.
    # Times are "seconds after bringup.launch.py starts".
    return LaunchDescription([
        # Start world + robots + localization immediately
        spawn_five_tb3,

        TimerAction(
            period = 5.0,
            actions = [multi_amcl]
        ),

        # Wait a bit, then start inflation
        TimerAction(
            period=2.0,
            actions=[inflation_costmap]
        ),

        # Then RViz and A*
        TimerAction(
            period=2.0,
            actions=[rviz]
        ),
        TimerAction(
            period=2.0,
            actions=[a_star_planner]
        ),

        # Then your PD motion planner
        TimerAction(
            period=2.0,
            actions=[pd_motion_planner]
        ),

        # Finally, path allocation (needs TF + map + A* ready)
        TimerAction(
            period=4.0,
            actions=[path_allocation]
        ),
    ])
