import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Paths to existing packages
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    warehouse_world_pkg = get_package_share_directory('warehouse_world')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='3.0')
    y_pose = LaunchConfiguration('y_pose', default='-13.0')
    z_pose = LaunchConfiguration('z_pose', default='0.15')
    world = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(warehouse_world_pkg, 'worlds', 'warehouse.world'),
        description='Full path to the world file to load'
    )

    declare_tb3_model = DeclareLaunchArgument(
        'tb3_model', default_value='burger',
        description='TurtleBot3 model type [burger, waffle, waffle_pi]'
    )

    # Environment variables (TB3 launch files expect these)
    set_tb3_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', LaunchConfiguration('tb3_model'))

    # Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        )
    )

    # TB3 robot_state_publisher
    robot_state_pub_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # TB3 spawn command (handles proper URDF, meshes, etc.)
    spawn_tb3_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose
        }.items()
    )

    # LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_tb3_model)
    ld.add_action(set_tb3_model_env)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_pub_cmd)
    ld.add_action(spawn_tb3_cmd)

    return ld
