from setuptools import setup, find_packages

setup(
    name='p2p_ros_package',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs'
    ],
    author='Your Name',
    author_email='you@example.com',
    description='ROS2 P2P package with CRUD/SQLite support',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'publisher_node = p2p_ros_package.main.publisher:main',
            'subscriber_node = p2p_ros_package.main.subscriber:main'
        ],
    },
)