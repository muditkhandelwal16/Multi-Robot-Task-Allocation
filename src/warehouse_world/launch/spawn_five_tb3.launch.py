import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node



def generate_launch_description():
    # Packages
    warehouse_pkg = get_package_share_directory('warehouse_world')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    # Launch files
    bringup_gazebo_launch = os.path.join(
        warehouse_pkg,
        'launch',
        'bringup_gazebo.launch.py'
    )
    robot_state_pub_launch = os.path.join(
        tb3_gazebo_pkg,
        'launch',
        'robot_state_publisher.launch.py'
    )

    # TurtleBot3 model environment variable (used by robot_state_publisher)
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # (namespace, x, y)
    robots = [
        ('tb3_1',  3.0,  -13.0),
        ('tb3_2',  4.5,  -13.0),
        ('tb3_3',  6.0,  -13.0),
        ('tb3_4',-3.5,  -13.0),
        ('tb3_5', -5.0,  -13.0),
    ]

    actions = [set_tb3_model]

    # 1) Start Gazebo + warehouse world
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_gazebo_launch)
        )
    )

    # 2) Spawn robots
    for ns, x, y in robots:
        # Per-robot SDF inside warehouse_world/models
        tb3_model_sdf = os.path.join(
            warehouse_pkg,
            'models',
            f'{ns}_burger',
            'model.sdf'
        )

        actions.append(
            GroupAction([
                # Namespace for this robot
                PushRosNamespace(ns),

                # TF + URDF publisher for this robot
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(robot_state_pub_launch),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'frame_prefix': ns,      # e.g. "tb3_1"
                    }.items()
                ),


                # Spawn this robot using its namespaced model
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name=f'{ns}_spawn_entity',
                    arguments=[
                        '-entity', ns,          # unique Gazebo entity name
                        '-file', tb3_model_sdf,
                        '-x', str(x),
                        '-y', str(y),
                        '-z', '0.01',
                    ],
                    output='screen'
                ),
            ])
        )

    return LaunchDescription(actions)
