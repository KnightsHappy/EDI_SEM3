cmake_minimum_required(VERSION 3.5)
project(ur5_control)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  src/task1b_boiler_plate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()