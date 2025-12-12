from setuptools import setup, find_packages

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # This tells ament where this package is indexed
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # This installs package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mudit',
    maintainer_email='muditmnr@gmail.com',
    description='Multi-robot path planning and task allocation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a_star_planner = path_planning.a_star_planner:main',
            'path_allocation = path_planning.path_allocation:main',
            'inflation_costmap = path_planning.inflation_costmap_node:main',
        ],
    },
)
