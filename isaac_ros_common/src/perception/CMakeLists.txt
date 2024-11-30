cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(builtin_interfaces REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pcl_conversions REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(OpenCV 4 REQUIRED)
find_package(std_msgs REQUIRED)

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# include_directories(
#     include
#     SYSTEM
#     ${EIGEN3_INCLUDE_DIRS}
#     ${PCL_INCLUDE_DIRS}
# )

add_executable(imu_orientation src/imu_orientation.cpp)
ament_target_dependencies(
    imu_orientation
    geometry_msgs
    std_msgs
    sensor_msgs
    builtin_interfaces
    rclcpp
    tf2
    tf2_ros
    tf2_eigen
    Eigen3
)

install(TARGETS
  imu_orientation
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(imu_integrator src/imu_integrator.cpp)
# ament_target_dependencies(
#     imu_integrator
#     geometry_msgs
#     sensor_msgs
#     rclcpp
#     tf2
#     tf2_ros
#     tf2_eigen
#     Eigen3
# )

# install(TARGETS
#     imu_integrator
#     DESTINATION lib/${PROJECT_NAME})

# add_executable(perception src/perception.cpp)
# ament_target_dependencies(
#     perception
#     geometry_msgs
#     sensor_msgs
#     rclcpp
#     tf2
#     tf2_ros
#     tf2_eigen
#     Eigen3
#     pcl_conversions
#     tf2_sensor_msgs
#     cv_bridge
#     image_transport
#     OpenCV
#     std_msgs
# )

# target_link_libraries(
#     perception
#     ${PCL_LIBRARIES}
# )

# install(TARGETS
#     perception
#     DESTINATION lib/${PROJECT_NAME})

ament_package()