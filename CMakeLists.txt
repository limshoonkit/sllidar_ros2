################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.22.1)
project(sllidar_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-function)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

set(SLLIDAR_SDK_PATH "./sdk/")

FILE(GLOB SLLIDAR_SDK_SRC 
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

add_library(sllidar_sdk STATIC ${SLLIDAR_SDK_SRC})
set_target_properties(sllidar_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(sllidar_sdk PUBLIC
  "${SLLIDAR_SDK_PATH}/include"
  "${SLLIDAR_SDK_PATH}/src"
)
################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  ${SLLIDAR_SDK_PATH}/include
  ${SLLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
)

add_executable(sllidar_node	src/sllidar_node.cpp ${SLLIDAR_SDK_SRC})
# target_link_libraries(sllidar_node ${ament_cmake_LIBRARIES})
ament_target_dependencies(sllidar_node
  rclcpp
  std_srvs
  sensor_msgs
)

add_executable(sllidar_client src/sllidar_client.cpp)
ament_target_dependencies(sllidar_client
  rclcpp
  std_srvs
  sensor_msgs
)

add_library(sllidar_component SHARED src/sllidar_component.cpp)
ament_target_dependencies(sllidar_component
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
)
target_link_libraries(sllidar_component sllidar_sdk)
rclcpp_components_register_nodes(sllidar_component "sllidar_ros2::SLlidarComponent")
################################################################################
# Install
################################################################################
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS sllidar_node sllidar_client sllidar_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
)
ament_package()
