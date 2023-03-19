cmake_minimum_required(VERSION 3.5)
project(tirrex_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/tirrex_demo.py)

install(PROGRAMS
  scripts/robot_description.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
