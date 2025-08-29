from setuptools import find_packages, setup

package_name = 'strategy_command_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabio',
    maintainer_email='fabio@example.com',
    description='ROS2 GUI for sending strategy commands to PathDriver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'strategy_gui = strategy_command_gui.strategy_gui:main',
        ],
    },
)
