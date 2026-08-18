cmake_minimum_required(VERSION 3.8)
project(secure_station_dispatch)

# ── Compiler warnings ──────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find packages ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# ── Generate custom message and action interfaces ──────────────────────────────
# NOTE: rosidl_generate_interfaces creates the Python bindings under
#       <install>/lib/python3.x/site-packages/secure_station_dispatch/
#       so the Python source package must be installed to a DIFFERENT path to
#       avoid target name collisions with ament_python_install_package().
#       We therefore install the Python source tree directly with install()
#       and set PYTHONPATH via ament_environment_hooks instead.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StationMarker.msg"
  "action/StationTask.action"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    action_msgs
)

# ── Install Python source package ─────────────────────────────────────────────
# Install the entire secure_station_dispatch/ Python tree into site-packages so
# "from secure_station_dispatch.security.checks import ..." works at runtime.
# We use install(DIRECTORY) rather than ament_python_install_package() because
# the latter clashes with rosidl's internal CMake target of the same name.
install(DIRECTORY
  secure_station_dispatch/
  DESTINATION lib/python3.12/site-packages/secure_station_dispatch
  FILES_MATCHING
    PATTERN "*.py"
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

# ── Install config, launch, worlds, models ────────────────────────────────────
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

# ── Install test / utility / attack scripts ────────────────────────────────────
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
  attacks/spoof_marker_publish.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spoof_marker_publish
)
install(PROGRAMS
  attacks/location_mismatch_attack.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME location_mismatch_attack
)
install(DIRECTORY
  attacks/
  DESTINATION share/${PROJECT_NAME}/attacks
)

# ── Install config, launch ────────────────────────────────────────────────────
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Export rosidl runtime dependency ─────────────────────────────────────────
ament_export_dependencies(rosidl_default_runtime)

ament_package()
