from setuptools import setup

package_name = 'wheel_controller'

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
    maintainer='clara',
    maintainer_email='handoyo.clara@gmail.com',
    description='A simple wheel controller for converting cmd_vel to wheel velocities.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_controller = wheel_controller.wheel_controller:main',
        ],
    },
)

