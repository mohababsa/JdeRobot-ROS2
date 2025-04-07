from setuptools import setup

package_name = 'waypoint_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed ABABSA',
    maintainer_email='mababsa80@gmail.com',
    description='Waypoint navigation for TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator = waypoint_nav.waypoint_navigator:main',
        ],
    },
)
