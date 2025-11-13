cmake_minimum_required(VERSION 3.5)
project(slam_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(ament_cmake_python REQUIRED) #for python scripts
find_package(rclcpp REQUIRED)
#find_package(rclpy REQUIRED) #for python scripts
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)


set(dependencies "rclcpp" "geometry_msgs" "nav_msgs" "tf2" "tf2_ros" "sensor_msgs" "pcl_ros" "pcl_conversions") 

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster ${dependencies})

add_executable(pc2_to_xyzi src/pc2_to_xyzi.cpp)
ament_target_dependencies(pc2_to_xyzi ${dependencies})

add_executable(lidar_imu_sync src/lidar_imu_sync.cpp)
ament_target_dependencies(lidar_imu_sync ${dependencies})

add_executable(pcd_visualizer src/pcd_visualizer.cpp)
ament_target_dependencies(pcd_visualizer ${dependencies})


install(TARGETS tf_broadcaster pc2_to_xyzi lidar_imu_sync pcd_visualizer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch urdf meshes rviz worlds src config scripts 
  DESTINATION share/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables #They must start with the shebag tag
install(PROGRAMS
  scripts/omni_teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME} )

ament_package()


