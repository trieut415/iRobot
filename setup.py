from setuptools import setup, find_packages

package_name = 'p2p_ros_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs'
    ],
    author='Trieu Tran',
    author_email='trieut@bu.edu',
    description='ROS2 P2P package with CRUD/SQLite support',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'publisher = p2p_ros_package.main.publisher:main',
            'subscriber = p2p_ros_package.main.subscriber:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['src/p2p_ros_package/launch/p2p_launch.py']),
    ],
    zip_safe=True,
)

