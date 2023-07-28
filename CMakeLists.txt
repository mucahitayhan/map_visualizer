cmake_minimum_required(VERSION 3.5)
project(map_visualizer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(lanelet2_core)
find_package(lanelet2_io)
find_package(lanelet2_maps)
find_package(lanelet2_projection)
find_package(lanelet2_routing)
find_package(lanelet2_traffic_rules)
find_package(lanelet2_validation)

set(dep_osm "rclcpp" "visualization_msgs" "Eigen3" "lanelet2_core" "lanelet2_io" "lanelet2_maps" "lanelet2_validation" "lanelet2_routing" "lanelet2_projection" "lanelet2_traffic_rules")

include_directories(include())

add_executable(osm_visualizer 
                src/osmVisualizer/osmVisualizer.cpp
                src/osmVisualizer/osm_visualizer_node.cpp)

ament_target_dependencies(osm_visualizer ${dep_osm})

target_include_directories(osm_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS osm_visualizer
  DESTINATION lib/${PROJECT_NAME})



########### pcd vis
find_package(PCL REQUIRED QUIET COMPONENTS common features filters io segmentation surface)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include())
include_directories(${PCL_INCLUDE_DIRS})

add_executable(pcd_visualizer src/pcdVisualizer/pcdVisualizer.cpp)
ament_target_dependencies(pcd_visualizer rclcpp sensor_msgs Eigen3 pcl_conversions tf2 tf2_geometry_msgs tf2_ros PCL)

target_include_directories(pcd_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(pcd_visualizer ${PCL_LIBRARIES})

install(TARGETS pcd_visualizer
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

### occupancy grid
find_package(nav_msgs REQUIRED)


include_directories(include())

add_executable(occupancy_pub src/occupancyGrid/occupancyGrip2.cpp)
ament_target_dependencies(occupancy_pub rclcpp nav_msgs)

target_include_directories(occupancy_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(occupancy_pub ${PCL_LIBRARIES})

install(TARGETS occupancy_pub
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
