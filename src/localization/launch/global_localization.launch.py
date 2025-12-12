import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")
    map_yaml = LaunchConfiguration("map_yaml")
    lifecycle_nodes = ["map_server", "amcl"]

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulated clock if true"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        # keep AMCL config in your package (adjust if you moved it)
        default_value="/home/mudit/turtlebot3_ws/install/localization/share/localization/config/amcl.yaml",
        description="Path to AMCL YAML"
    )

    # DEFAULT: absolute path to your home map
    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value="/home/mudit/map.yaml",
        description="Absolute path to map.yaml"
    )

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml},
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        amcl_config_arg,
        map_yaml_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,
    ])
