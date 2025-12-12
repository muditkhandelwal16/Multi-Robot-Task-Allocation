from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {"yaml_filename": "/home/mudit/map.yaml"},
                {"frame_id": "map"},          # explicit, just in case
                {"use_sim_time": True},       # set False on the real robot
            ],
        )
    ])
