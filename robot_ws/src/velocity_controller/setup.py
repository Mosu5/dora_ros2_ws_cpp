from setuptools import setup
import os
import glob

package_name = 'velocity_controller'

# Find all launch files
launch_files = glob.glob(os.path.join('launch', '*'))

# Find all description files
description_files = glob.glob(os.path.join('description', '*'))

setup(
    name=package_name,
    version='0.0.0',
    packages=['velocity_controller'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the launch files in the data_files section
        (os.path.join('share', package_name, 'launch'), launch_files),
        # Include the description files in the data_files section
        (os.path.join('share', package_name, 'description'), description_files),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='moustapha',
    maintainer_email='moustapha@example.com',
    description='Velocity controller package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_controller = velocity_controller.velocity_controller:main',
        ],
    },
)