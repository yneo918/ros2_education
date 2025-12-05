"""Setup configuration for ros2_edu_util package."""

from setuptools import find_packages, setup

package_name = 'ros2_edu_util'

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
    description='Common ROS2 utilities: PubSubManager, JoyBase, NavNode',
    license='ECL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
