launch the world
ros2 launch warehouse_world bringup_gazebo.launch.py

spawn turtlebot
ros2 launch warehouse_world spawn_only_tb3.launch.py

push map
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/mudit/map.yaml
ros2 lifecycle set /map_server 1
ros2 lifecycle set /map_server 3

give goal to /goal_pose (manual for now)
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"

run a_star
ros2 run bumperbot_motion a_star_planner.py

amcl node
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

then run pd_modion 
ros2 run bumperbot_motion pd_modion_planner.py