from setuptools import setup

package_name = 'hello_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed ABABSA',
    maintainer_email='mababsa80@gmail.com',
    description='ROS2 Publisher-Subscriber Example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = hello_ros2.publisher_node:main',
            'subscriber_node = hello_ros2.subscriber_node:main',
        ],
    },
)
