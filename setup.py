from setuptools import setup

package_name = 'waypoint_autonavigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/map_collision_injector_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ciprian Dumitrache and Andreea Stratulat',
    maintainer='Ciprian Dumitrache and Andreea Stratulat',
    maintainer_email='email',
    description='Waypoint navigation for autonomous robots in ROS2.',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'waypoint_autonavigator = waypoint_autonavigator.main:main',
            'map_collision_injector = waypoint_autonavigator.map_collision_injector:main',
        ],
    },
)

