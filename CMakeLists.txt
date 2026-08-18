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
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StationMarker.msg"
  "action/StationTask.action"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    action_msgs
)

# ── Install Python package (secure_station_dispatch/) ─────────────────────────
ament_python_install_package(${PROJECT_NAME})

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

# ── Install test / utility scripts ────────────────────────────────────────────
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
