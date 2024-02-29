cmake_minimum_required(VERSION 3.8)
project(rto_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2)
find_package(tf2_ros)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

#set(CMAKE_MODULE_PATH ~/ros2_ws/install/robotino_msgs/share/robotino_msgs/cmake)
find_package(rto_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_MODULE_PATH /opt/robotino/cmake)
find_package(RobotinoAPI2)

set(CMAKE_MODULE_PATH /opt/rec_rpc/cmake)
find_package(RecRpc)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/robotino_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/RTONode.cpp
  src/ComROS.cpp
  src/PowerManagementROS.cpp
  src/OmniDriveROS.cpp
  src/GyroscopeROS.cpp
  src/MotorArrayROS.cpp
  src/BumperROS.cpp
  src/AnalogInputArrayROS.cpp
  src/DigitalInputArrayROS.cpp
  src/DigitalOutputArrayROS.cpp
  src/DistanceSensorArrayROS.cpp
  src/ElectricalGripperROS.cpp
  src/EncoderInputROS.cpp
)

target_link_libraries(${PROJECT_NAME} ${REC_ROBOTINO_API2_LIBRARY})
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs geometry_msgs std_msgs rto_msgs)

add_executable(rto_odometry_node
  src/rto_odometry_node.cpp
  src/RTOOdometryNode.cpp
  src/ComROS.cpp
  src/OdometryROS.cpp
)
target_link_libraries(rto_odometry_node ${REC_ROBOTINO_API2_LIBRARY})
ament_target_dependencies(rto_odometry_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros rto_msgs)

add_executable(rto_laserrangefinder_node
  src/rto_laserrangefinder_node.cpp
  src/ComROS.cpp
  src/LaserRangeFinderROS.cpp
  src/RTOLaserRangeFinderNode.cpp
)
target_link_libraries(rto_laserrangefinder_node ${REC_ROBOTINO_API2_LIBRARY})
ament_target_dependencies(rto_laserrangefinder_node rclcpp sensor_msgs)

add_executable(rto_camera_node
  src/rto_camera_node.cpp
  src/RTOCameraNode.cpp
  src/ComROS.cpp
  src/CameraROS.cpp
)
target_link_libraries(rto_camera_node ${REC_ROBOTINO_API2_LIBRARY})
ament_target_dependencies(rto_camera_node rclcpp std_msgs sensor_msgs)

install(
  TARGETS ${PROJECT_NAME} rto_odometry_node rto_camera_node rto_laserrangefinder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
