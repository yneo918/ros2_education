"""Setup configuration for virtual_joy package."""

from setuptools import find_packages, setup

package_name = 'virtual_joy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS2 Education Project',
    maintainer_email='ros2edu@example.com',
    description='PyQt6-based virtual Xbox controller publishing Joy messages',
    license='ECL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_joy = virtual_joy.virtual_joy:main'
        ],
    },
)
