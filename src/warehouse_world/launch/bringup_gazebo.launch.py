from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg = FindPackageShare('warehouse_world')
    world = PathJoinSubstitution([pkg, 'worlds', 'warehouse.world'])
    models_dir = PathJoinSubstitution([pkg, 'models'])

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            TextSubstitution(text=':'),
            models_dir
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('gazebo_ros'), TextSubstitution(text='/launch/gazebo.launch.py')]
        ),
        launch_arguments={'world': world}.items()
    )

    return LaunchDescription([set_model_path, gazebo])
