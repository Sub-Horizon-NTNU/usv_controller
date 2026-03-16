cmake_minimum_required(VERSION 3.8)
project(usv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(waypoint_msgs REQUIRED)

add_executable(usv_controller src/main.cpp
src/Controller.cpp
src/PID.cpp
src/USV.cpp
src/Waypoint.cpp
src/PathScheduler.cpp
)
target_include_directories(usv_controller PRIVATE ${CMAKE_SOURCE_DIR}/include)

ament_target_dependencies(usv_controller
  rclcpp
  mavros
  mavros_msgs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
  waypoint_msgs
)

install(TARGETS usv_controller
DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
