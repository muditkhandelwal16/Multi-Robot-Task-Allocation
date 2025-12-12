from setuptools import setup

package_name = 'motion_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # install launch files
        ('share/' + package_name + '/launch', ['launch/pd_motion_planner_multi.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mudit',
    maintainer_email='mudit@example.com',
    description='PD motion planner for multi-robot navigation with TurtleBot3',
    license='MIT',

    # Register the Python executable
    entry_points={
        'console_scripts': [
            'pd_motion_planner = motion_planner.pd_motion_planner:main',
        ],
    },
)
