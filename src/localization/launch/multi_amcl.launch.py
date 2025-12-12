import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # --- Launch args ---
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map_yaml")
    amcl_config = LaunchConfiguration("amcl_config")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulated time",
    )

    declare_map_yaml = DeclareLaunchArgument(
        "map_yaml",
        default_value="/home/mudit/map.yaml",
        description="Absolute path to map.yaml",
    )

    declare_amcl_config = DeclareLaunchArgument(
        "amcl_config",
        default_value=(
            "/home/mudit/turtlebot3_ws/install/localization/"
            "share/localization/config/amcl.yaml"
        ),
        description="Path to AMCL YAML",
    )

    # --- Shared map_server (no namespace) ---
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml},
            {"use_sim_time": use_sim_time},
        ],
    )

    # --- Lifecycle manager for map_server ---
    map_lifecycle_mgr = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"node_names": ["map_server"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
        ],
    )

    robots = [("tb3_1", 3.0, -13.0, 0.0),
        ("tb3_2", 4.5, -13.0, 0.0),
        ("tb3_3", 6.0, -13.0, 0.0),
        ("tb3_4", -3.5, -13.0, 0.0),
        ("tb3_5", -5.0, -13.0, 0.0),
    ]
    
    amcl_groups = []

    for ns, init_x, init_y, init_yaw in robots:
        group = GroupAction([
            PushRosNamespace(ns),

            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[
                    amcl_config,
                    {"use_sim_time": use_sim_time},

                    # Per-robot initial pose
                    {"set_initial_pose": True},
                    {
                        "initial_pose": {
                            "x": init_x,
                            "y": init_y,
                            "z": 0.0,
                            "yaw": init_yaw,
                        }
                    },


                    {"base_frame_id": f"{ns}/base_footprint"},
                    {"odom_frame_id": f"{ns}/odom"},
                    {"global_frame_id": "map"},

                    # AMCL subscribes to "scan" (we remap it below)
                    {"scan_topic": "scan"},
                ],
                remappings=[
                    ("scan", f"/{ns}/scan"),
                    ("map", "/map"),
                ],
            ),

            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": ["amcl"]},
                ],
            ),
        ])


        amcl_groups.append(group)

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        declare_amcl_config,
        map_server,
        map_lifecycle_mgr,
        *amcl_groups,
    ])