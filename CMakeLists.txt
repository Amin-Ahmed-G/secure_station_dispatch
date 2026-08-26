cmake_minimum_required(VERSION 3.8)
project(secure_station_dispatch)

# ── Compiler warnings ─────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find packages ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# ── Generate custom message and action interfaces ─────────────────────────────
# NOTE: rosidl_generate_interfaces creates the Python bindings under
#       <install>/lib/python3.x/site-packages/secure_station_dispatch/
#       so the Python source package must be installed to the SAME path but via
#       plain install() — ament_python_install_package() would declare a CMake
#       target of the same name as rosidl's and the configure step would fail.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StationMarker.msg"
  "action/StationTask.action"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    action_msgs
)

# ── Install the Python source package ─────────────────────────────────────────
# install(DIRECTORY) recurses, so secure_station_dispatch/security/ and
# secure_station_dispatch/navigation/ come along automatically.  The interpreter
# version is queried rather than hard-coded so the package builds unchanged on
# Humble (3.10) and Jazzy (3.12).
set(PYTHON_SITE_PACKAGES
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

install(DIRECTORY
  secure_station_dispatch/
  DESTINATION ${PYTHON_SITE_PACKAGES}/secure_station_dispatch
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
)

# ── Install Python node executables ───────────────────────────────────────────
install(PROGRAMS
  secure_station_dispatch/marker_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME marker_detector
)
install(PROGRAMS
  secure_station_dispatch/dispatcher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dispatcher
)
install(PROGRAMS
  secure_station_dispatch/task_simulator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME task_simulator
)
install(PROGRAMS
  secure_station_dispatch/waypoint_tour_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_tour
)

# ── Install test / utility / attack scripts ───────────────────────────────────
install(PROGRAMS
  test/mock_image_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mock_image_publisher
)
install(PROGRAMS
  test/test_dispatch_pipeline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_dispatch_pipeline
)
install(PROGRAMS
  test/test_security_checks.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_security_checks
)
install(PROGRAMS
  test/test_waypoint_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_waypoint_planner
)
install(PROGRAMS
  test/test_tour_simulation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_tour_simulation
)
install(PROGRAMS
  attacks/spoof_marker_publish.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spoof_marker_publish
)
install(PROGRAMS
  attacks/location_mismatch_attack.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME location_mismatch_attack
)

# ── Install resources ─────────────────────────────────────────────────────────
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY worlds/  DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/  DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY attacks/ DESTINATION share/${PROJECT_NAME}/attacks)
install(DIRECTORY test/    DESTINATION share/${PROJECT_NAME}/test)
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts
        USE_SOURCE_PERMISSIONS)
install(DIRECTORY docs/    DESTINATION share/${PROJECT_NAME}/docs OPTIONAL)

# ── Tests ─────────────────────────────────────────────────────────────────────
# The navigation regression suites are ROS-free, so colcon test can run them
# directly.  They are what pin the Phase 3 bay-collision fix.
if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_waypoint_planner test/test_waypoint_planner.py
                          TIMEOUT 120)
    ament_add_pytest_test(test_tour_simulation test/test_tour_simulation.py
                          TIMEOUT 120)
  endif()
endif()

# ── Export rosidl runtime dependency ──────────────────────────────────────────
ament_export_dependencies(rosidl_default_runtime)

ament_package()
