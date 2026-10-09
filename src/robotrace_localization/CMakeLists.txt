cmake_minimum_required(VERSION 3.10)
project(robotrace_localization)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs sensor_msgs
  std_msgs std_srvs diagnostic_msgs tf2_ros message_filters pcl_ros pcl_conversions
  ublox_msgs fast_gicp)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
find_package(CUDA REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
if(NOT BUILD_VGICP_CUDA)
  message(FATAL_ERROR "robotrace_localization requires -DBUILD_VGICP_CUDA=ON")
endif()
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp geometry_msgs nav_msgs
  sensor_msgs std_msgs std_srvs diagnostic_msgs tf2_ros message_filters
  pcl_ros pcl_conversions ublox_msgs fast_gicp)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(gnss_initializer_node src/gnss_initializer_node.cpp)
target_link_libraries(gnss_initializer_node ${catkin_LIBRARIES})
add_dependencies(gnss_initializer_node ${catkin_EXPORTED_TARGETS})
add_executable(gpu_map_localizer_node src/gpu_map_localizer_node.cpp)
target_compile_definitions(gpu_map_localizer_node PRIVATE USE_VGICP_CUDA)
target_link_libraries(gpu_map_localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES}
  ${CUDA_CUDART_LIBRARY} OpenMP::OpenMP_CXX Threads::Threads)
add_dependencies(gpu_map_localizer_node fast_gicp fast_vgicp_cuda ${catkin_EXPORTED_TARGETS})

# These ROS-independent checks have their own main functions and do not need
# GTest. `catkin_make tests` builds them; CTest or scripts/test.sh runs them.
if(CATKIN_ENABLE_TESTING)
  add_executable(gnss_seed_test EXCLUDE_FROM_ALL ../../tests/gnss_seed_test.cpp)
  add_executable(pose_math_test EXCLUDE_FROM_ALL ../../tests/pose_math_test.cpp)
  add_dependencies(tests gnss_seed_test pose_math_test)
  add_test(NAME robotrace_gnss_seed COMMAND $<TARGET_FILE:gnss_seed_test>)
  add_test(NAME robotrace_pose_math COMMAND $<TARGET_FILE:pose_math_test>)
endif()

install(TARGETS gnss_initializer_node gpu_map_localizer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
