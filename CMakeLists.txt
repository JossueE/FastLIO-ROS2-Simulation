cmake_minimum_required(VERSION 3.5)
project(slam_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)   # para ament_python_install_package

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

find_package(ndt_omp_ros2 REQUIRED)
find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ------------------------------------------------------------------------------
# Include dirs
# ------------------------------------------------------------------------------

# Headers propios
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ------------------------------------------------------------------------------
# Librería: lidar_localization_component
# ------------------------------------------------------------------------------

add_library(lidar_localization_component SHARED
  src/lidar_localization_component.cpp
  # añade aquí más .cpp relacionados si quieres que formen parte de la misma lib
  # src/lidar_undistortion.cpp
)

target_include_directories(lidar_localization_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lidar_localization_component
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  ndt_omp_ros2
)

target_link_libraries(lidar_localization_component
  ${PCL_LIBRARIES}
)

ament_export_libraries(lidar_localization_component)

# ------------------------------------------------------------------------------
# Dependencias comunes para los nodos
# ------------------------------------------------------------------------------

set(common_dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
  ndt_omp_ros2
)

# ------------------------------------------------------------------------------
# Ejecutables
# ------------------------------------------------------------------------------

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster ${common_dependencies})

add_executable(pc2_to_xyzi src/pc2_to_xyzi.cpp)
ament_target_dependencies(pc2_to_xyzi ${common_dependencies})
target_link_libraries(pc2_to_xyzi ${PCL_LIBRARIES})

add_executable(lidar_imu_sync src/lidar_imu_sync.cpp)
ament_target_dependencies(lidar_imu_sync ${common_dependencies})
target_link_libraries(lidar_imu_sync ${PCL_LIBRARIES})

add_executable(pcd_visualizer src/pcd_visualizer.cpp)
ament_target_dependencies(pcd_visualizer ${common_dependencies})
target_link_libraries(pcd_visualizer ${PCL_LIBRARIES})

add_executable(lidar_localization_node src/lidar_localization_node.cpp)
ament_target_dependencies(lidar_localization_node ${common_dependencies})
target_link_libraries(lidar_localization_node
  lidar_localization_component
  ${PCL_LIBRARIES}
)

# OJO: ya NO creamos un ejecutable llamado igual que la librería
# (antes tenías: add_executable(lidar_localization_component ...), eso se elimina)

# ------------------------------------------------------------------------------
# Instalación
# ------------------------------------------------------------------------------

install(TARGETS
  tf_broadcaster
  pc2_to_xyzi
  lidar_imu_sync
  pcd_visualizer
  lidar_localization_node
  lidar_localization_component
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  meshes
  rviz
  worlds
  config
  scripts
  include
  DESTINATION share/${PROJECT_NAME}
)

# Python package (directorio slam_sim/)
ament_python_install_package(${PROJECT_NAME})

# Ejecutables Python
install(PROGRAMS
  scripts/omni_teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
