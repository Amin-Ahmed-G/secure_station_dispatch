# NOTE: this package builds with ament_cmake (see CMakeLists.txt), which is what
# actually installs the nodes, interfaces and resources.  setup.py is kept only
# so the Python tree can be installed with pip for editor tooling and for running
# the ROS-free regression suites outside a workspace:
#
#     pip install -e .
#     python3 test/test_waypoint_planner.py
#     python3 test/test_tour_simulation.py
#
# Keep the package list and data_files in step with CMakeLists.txt.
from setuptools import setup, find_packages

package_name = 'secure_station_dispatch'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/station_registry.yaml']),
        ('share/' + package_name + '/launch', ['launch/station_dispatch.launch.py', 'launch/gazebo_demo.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/industrial_station_world.sdf']),
        ('share/' + package_name + '/models/materials/textures', [
            'models/materials/textures/marker_0.png',
            'models/materials/textures/marker_1.png',
            'models/materials/textures/marker_2.png',
            'models/materials/textures/marker_3.png',
            'models/materials/textures/nameplate_0.png',
            'models/materials/textures/nameplate_1.png',
            'models/materials/textures/nameplate_2.png',
            'models/materials/textures/nameplate_3.png',
            'models/materials/textures/infoplate_0.png',
            'models/materials/textures/infoplate_1.png',
            'models/materials/textures/infoplate_2.png',
            'models/materials/textures/infoplate_3.png',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='AMR station dispatch with ArUco marker detection, obstacle-aware\n                 navigation and ROS 2 action-based task execution',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_detector        = secure_station_dispatch.marker_detector_node:main',
            'dispatcher             = secure_station_dispatch.dispatcher_node:main',
            'task_simulator         = secure_station_dispatch.task_simulator_node:main',
            'waypoint_tour          = secure_station_dispatch.waypoint_tour_node:main',
            'mock_image_publisher   = test.mock_image_publisher:main',
            'test_dispatch_pipeline = test.test_dispatch_pipeline:main',
            'test_security_checks   = test.test_security_checks:main',
            'test_waypoint_planner  = test.test_waypoint_planner:main',
            'test_tour_simulation   = test.test_tour_simulation:main',
            'spoof_marker_publish   = attacks.spoof_marker_publish:main',
            'location_mismatch_attack = attacks.location_mismatch_attack:main',
        ],
    },
)
