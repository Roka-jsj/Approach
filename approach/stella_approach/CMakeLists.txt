cmake_minimum_required(VERSION 3.5)
project(stella_approach)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "**pycache**" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/approach_goal_service.py
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ApproachTarget.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
