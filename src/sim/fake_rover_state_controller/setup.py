"""
Setup configuration for fake_rover_state_controller package.

This is a standard ROS2 Python package setup file. Key elements:
- name: Package name (must match package.xml and folder name)
- packages: Python packages to include
- data_files: Non-Python files to install (package.xml, launch files, etc.)
- entry_points: Executable nodes registered with ROS2
"""

from setuptools import find_packages, setup

package_name = 'fake_rover_state_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),

    # Install non-Python files
    # Format: (install_destination, [source_files])
    data_files=[
        # Ament resource index marker (required for ROS2 to find the package)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    # Package metadata
    maintainer='ROS2 Education Project',
    maintainer_email='ros2edu@example.com',
    description='Simulated rover state controller with physics dynamics for ROS2 education',
    license='ECL-2.0',

    tests_require=['pytest'],

    # Register executable nodes
    # Format: 'executable_name = package.module:function'
    entry_points={
        'console_scripts': [
            'sim_rover = fake_rover_state_controller.sim_rover:main',
        ],
    },
)
