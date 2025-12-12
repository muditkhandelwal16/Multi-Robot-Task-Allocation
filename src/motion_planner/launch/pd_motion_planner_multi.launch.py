from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Launch 5 instances of the PD motion planner, one per TurtleBot3.

    IMPORTANT: replace `your_package_name` and `pd_motion_planner`
    with what you actually use in `ros2 run`:
      ros2 run <your_package_name> <your_executable>
    """
    robots = ["tb3_1", "tb3_2", "tb3_3", "tb3_4", "tb3_5"]

    nodes = []
    for rid in robots:
        nodes.append(
            Node(
                package="motion_planner",           # <-- change this
                executable="pd_motion_planner",        # <-- change this if needed
                name=f"{rid}_pd_motion_planner",
                parameters=[
                    {"odom_frame": f"{rid}/odom"},
                    {"base_frame": f"{rid}/base_footprint"},
                    {"path_topic": f"/{rid}/path"},
                    {"cmd_vel_topic": f"/{rid}/cmd_vel"},
                    {"next_pose_topic": f"/{rid}/pd/next_pose"},
                    # Optional: tune PD gains per robot if you want
                    # {"kp": 2.0},
                    # {"kd": 0.1},
                ],
                output="screen",
            )
        )

    return LaunchDescription(nodes)
