from setuptools import find_packages, setup

package_name = 'strategy'

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
    maintainer='geffte',
    maintainer_email='geffte.caetano@ufms.br',
    description='Hierarchical strategy system for robot soccer using Plays → Tactics → Skills architecture',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'strategy = strategy.strategy:main',
            'strategy_examples = strategy.examples:main',
        ],
    },
)
