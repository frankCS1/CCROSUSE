from setuptools import setup

package_name = 'my_robot_vision_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Frank Motsi',
    maintainer_email='n1007535@my.ntu.ac.uk',
    description='Robot navigation, SLAM and vision project for COMP30271',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follow_node = my_robot_vision_pkg.wall_follow_node:main',
            'goal_node = my_robot_vision_pkg.goal_node:main',
            'nav_node = my_robot_vision_pkg.nav_node:main',
            'control_node = my_robot_vision_pkg.control_node:main',
            'traffic_node = my_robot_vision_pkg.traffic_node:main',
            'landmark_logger = my_robot_vision_pkg.landmark_logger:main',
        ],
    },
)
