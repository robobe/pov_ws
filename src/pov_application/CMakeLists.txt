cmake_minimum_required(VERSION 3.8)
project(pov_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})


set (NODES
  pov_application/mav_node.py
  pov_application/odom_node.py
  pov_application/odom_to_velocity.py
  pov_application/odom_to_position.py
  pov_application/aruco_node.py
)

set (FILES
pov_application/mavlink.py
pov_application/pov_node.py
)

install(PROGRAMS
    ${NODES}
    ${FILES}
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
