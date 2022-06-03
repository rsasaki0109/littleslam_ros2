cmake_minimum_required(VERSION 3.5)
project(littleslam_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3)
find_package (PCL 1.8 REQUIRED)

add_subdirectory(./src/LittleSLAM/framework)
add_subdirectory(./src/LittleSLAM/hook)
add_subdirectory(./src/LittleSLAM/cui)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  src/LittleSLAM/framework
  src/LittleSLAM/hook
  src/LittleSLAM/cui
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(littleslam_ros2_component SHARED src/littleslam_ros2_component.cpp)

target_compile_definitions(littleslam_ros2_component PRIVATE "LITTLESLAM_BUILDING_DLL")
ament_target_dependencies(littleslam_ros2_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  class_loader
  )

rclcpp_components_register_nodes(
  littleslam_ros2_component "littleslam_ros2::Littleslam"
)

add_executable(${PROJECT_NAME}
    src/littleslam_ros2.cpp
    src/littleslam_ros2_component.cpp
    src/LittleSLAM/framework/SlamFrontEnd.cpp
    src/LittleSLAM/cui/FrameworkCustomizer.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_component
  ${PCL_LIBRARIES}
  framework
  hook
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp 
  sensor_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs   
)

install(TARGETS
  littleslam_ros2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(TARGETS
  littleslam_ros2
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
