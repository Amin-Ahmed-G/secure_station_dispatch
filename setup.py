from setuptools import setup, find_packages

package_name = 'secure_station_dispatch'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/station_registry.yaml']),
        ('share/' + package_name + '/launch', ['launch/station_dispatch.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='AMR station dispatch with ArUco marker detection and ROS2 actions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_detector    = secure_station_dispatch.marker_detector_node:main',
            'dispatcher         = secure_station_dispatch.dispatcher_node:main',
            'task_simulator     = secure_station_dispatch.task_simulator_node:main',
            'mock_image_publisher = test.mock_image_publisher:main',
            'test_dispatch_pipeline = test.test_dispatch_pipeline:main',
        ],
    },
)
