cmake_minimum_required(VERSION 3.8)
project(fix_distortion_fisheye_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  
find_package(tf2_sensor_msgs REQUIRED)
find_package(realsense2 REQUIRED)

# Include directories for C++ headers
include_directories(include)

# Add C++ executable for t265_node
add_executable(t265_node src/t265_node.cpp)
ament_target_dependencies(t265_node 
  rclcpp 
  nav_msgs 
  sensor_msgs 
  tf2_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs  
  tf2_sensor_msgs
  cv_bridge
  OpenCV
  realsense2
)
target_link_libraries(t265_node 
  ${OpenCV_LIBS} 
  ${cv_bridge_LIBRARIES}
  ${realsense2_LIBRARY}
)



# Install C++ executables
install(TARGETS
  t265_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package (ensures Python modules are found)
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (scripts)
install(PROGRAMS
  scripts/defisheye_fix_distortion.py
  scripts/defisheye_rectilinear_projection.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
