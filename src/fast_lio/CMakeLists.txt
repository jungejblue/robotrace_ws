cmake_minimum_required(VERSION 3.16)
project(fast_lio LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs nav_msgs sensor_msgs std_msgs tf
  pcl_ros pcl_conversions message_generation)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common filters kdtree)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
add_message_files(FILES Pose6D.msg)
generate_messages(DEPENDENCIES geometry_msgs)
catkin_package(CATKIN_DEPENDS
  roscpp geometry_msgs nav_msgs sensor_msgs std_msgs tf pcl_ros
  pcl_conversions message_runtime
  DEPENDS EIGEN3 PCL)

# Limit FAST-LIO nearest-neighbor parallel work to three threads on both
# x86_64 and Jetson. CUDA registration is owned by robotrace_localization.
set(FAST_LIO_THREADS 3 CACHE STRING "FAST-LIO OpenMP worker count")
if(FAST_LIO_THREADS LESS 1)
  message(FATAL_ERROR "FAST_LIO_THREADS must be at least 1")
endif()
add_executable(fastlio_mapping
  src/laserMapping.cpp src/preprocess.cpp include/ikd-Tree/ikd_Tree.cpp)
add_dependencies(fastlio_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(fastlio_mapping PRIVATE
  include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_compile_definitions(fastlio_mapping PRIVATE MP_PROC_NUM=${FAST_LIO_THREADS})
if(FAST_LIO_THREADS GREATER 1)
  target_compile_definitions(fastlio_mapping PRIVATE MP_EN)
endif()
target_compile_options(fastlio_mapping PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(fastlio_mapping
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX Threads::Threads)
install(TARGETS fastlio_mapping RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES LICENSE NOTICE UPSTREAM.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
