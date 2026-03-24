from setuptools import find_packages, setup

package_name = 'drillship_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ps5_teleop.launch.py']),
        ('share/' + package_name + '/launch', ['launch/dp_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kris',
    maintainer_email='krishmy@stud.ntnu.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
    'ta_node = drillship_sim.ta_node:main',
    'ps5_node = drillship_sim.ps5_node:main',
    'npo_node = drillship_sim.npo_node:main',
    'ps5_node_basin = drillship_sim.ps5_node_basin:main',
    'input_node = drillship_sim.input_node:main',
    'pathplanner_node = drillship_sim.pathplanner_node:main',
    'controller_node = drillship_sim.controller_node:main',
    'drillship_utility_node = drillship_sim.drillship_utility_node:main',
        ],
    },
)
