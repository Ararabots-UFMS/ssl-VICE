from setuptools import find_packages, setup

package_name = 'referee'

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
    maintainer='fernando',
    maintainer_email='fernando@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'referee_node = referee.referee_node:main',
            'talker = referee.publisher_member_function:main',
            'listener = referee.subscriber_member_function:main',
            'subscriber = referee.referee_subscriber:main'
        ],
    },
)
