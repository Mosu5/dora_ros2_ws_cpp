from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='moustapha',
    maintainer_email='m.azaimi@student.avans.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.minimal_publisher:main',
            'listener = py_pubsub.minimal_subscriber:main',
            'service = py_pubsub.minimal_service:main',
            'client = py_pubsub.minimal_client_async:main',
        ],
    },
)
