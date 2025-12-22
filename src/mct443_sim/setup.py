from setuptools import setup

package_name = 'mct443_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/spawn_robot.launch.py', 'launch/spawn_robot_launch2.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohammedaljamal',
    maintainer_email='',
    description='MCT443 lane keeping simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'drive_10m = mct443_sim.drive_10m:main',
        'lidar_debug = mct443_sim.lidar_debug:main',
        'drive_10m_omega = mct443_sim.drive_10m_omega:main',
        'lane_follow_node = mct443_sim.lane_follow_node:main',


        # add other nodes here later
    ],
    },
)

