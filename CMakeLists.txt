cmake_minimum_required(VERSION 3.8)
project(kaz_j25_ajr)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(AMENT_DEPS
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(world_node src/world_node.cpp)
ament_target_dependencies(world_node ${AMENT_DEPS})
target_link_libraries(world_node Qt5::Widgets)

add_executable(vacuum_node src/vacuum_node.cpp)
ament_target_dependencies(vacuum_node ${AMENT_DEPS})

install(TARGETS
  world_node
  vacuum_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
