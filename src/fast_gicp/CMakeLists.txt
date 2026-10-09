cmake_minimum_required(VERSION 3.10)
project(fast_gicp)

# ROS1 production-library build of the pinned upstream implementation.
# Applications, datasets, Python bindings and experimental implementations are omitted.
option(BUILD_VGICP_CUDA "Build the required CUDA VGICP backend" ON)
option(BUILD_apps "Unavailable in this focused source bundle" OFF)
option(BUILD_test "Unavailable in this focused source bundle" OFF)
option(BUILD_PYTHON_BINDINGS "Unavailable in this focused source bundle" OFF)
if(NOT BUILD_VGICP_CUDA)
  message(FATAL_ERROR "This workspace requires BUILD_VGICP_CUDA=ON")
endif()
if(BUILD_apps OR BUILD_test OR BUILD_PYTHON_BINDINGS)
  message(FATAL_ERROR "This focused fast_gicp bundle provides library targets only")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(ROBOTRACE_CUDA_ARCH "120" CACHE STRING "GPU compute capability without a dot")
if(NOT ROBOTRACE_CUDA_ARCH MATCHES "^[0-9][0-9][0-9]?$")
  message(FATAL_ERROR "Use ROBOTRACE_CUDA_ARCH=120 for RTX 5060 Ti, or 87 for Orin")
endif()

find_package(catkin REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree search registration)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(CUDA REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(${CUDA_INCLUDE_DIRS})
# FindCUDA does not consume CMAKE_CUDA_ARCHITECTURES. Preserve the working
# CUDA 12.8 / GCC 10 path and emit native code for the selected GPU explicitly.
list(APPEND CUDA_NVCC_FLAGS
  "--expt-relaxed-constexpr"
  "-gencode=arch=compute_${ROBOTRACE_CUDA_ARCH},code=sm_${ROBOTRACE_CUDA_ARCH}")
add_definitions(-DUSE_VGICP_CUDA)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fast_gicp fast_vgicp_cuda
  DEPENDS PCL Eigen3
)

cuda_add_library(fast_vgicp_cuda SHARED
  src/fast_gicp/cuda/fast_vgicp_cuda.cu
  src/fast_gicp/cuda/brute_force_knn.cu
  src/fast_gicp/cuda/covariance_estimation.cu
  src/fast_gicp/cuda/covariance_estimation_rbf.cu
  src/fast_gicp/cuda/covariance_regularization.cu
  src/fast_gicp/cuda/gaussian_voxelmap.cu
  src/fast_gicp/cuda/find_voxel_correspondences.cu
  src/fast_gicp/cuda/compute_derivatives.cu
  src/fast_gicp/cuda/compute_mahalanobis.cu
  src/fast_gicp/cuda/ndt_cuda.cu
  src/fast_gicp/cuda/ndt_compute_derivatives.cu
)
target_include_directories(fast_vgicp_cuda PRIVATE
  include
  thirdparty/Eigen
  thirdparty/nvbio
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(fast_vgicp_cuda ${CUDA_CUDART_LIBRARY})
cuda_add_cublas_to_target(fast_vgicp_cuda)

add_library(fast_gicp SHARED
  src/fast_gicp/gicp/lsq_registration.cpp
  src/fast_gicp/gicp/fast_gicp.cpp
  src/fast_gicp/gicp/fast_gicp_st.cpp
  src/fast_gicp/gicp/fast_vgicp.cpp
  src/fast_gicp/gicp/fast_vgicp_cuda.cpp
  src/fast_gicp/ndt/ndt_cuda.cpp
)
target_compile_definitions(fast_gicp PUBLIC USE_VGICP_CUDA)
target_include_directories(fast_gicp PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(fast_gicp PUBLIC
  fast_vgicp_cuda ${PCL_LIBRARIES} OpenMP::OpenMP_CXX
)
add_dependencies(fast_gicp fast_vgicp_cuda)

install(TARGETS fast_gicp fast_vgicp_cuda
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(FILES LICENSE UPSTREAM_MANIFEST.json UPSTREAM.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
