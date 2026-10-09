# Pinned registration implementation

The production fast_gicp sources are bundled from
[`koide3/fast_gicp` commit `0e7ec1441c99f7be453db2ea216d5de029387417`](https://github.com/koide3/fast_gicp/tree/0e7ec1441c99f7be453db2ea216d5de029387417).
No Git clone or submodule update is required to build this workspace.

The library keeps the production upstream APIs, including
`fast_gicp::FastVGICPCuda<pcl::PointXYZI, pcl::PointXYZI>`. It exports the
`fast_gicp` and `fast_vgicp_cuda` shared-library targets through catkin.
The localization node selects VGICP; the upstream CUDA NDT sources remain
compiled to preserve the library build that succeeded on the lab desktop.

Applied integration fixes:

- Explicit `-gencode` using `ROBOTRACE_CUDA_ARCH`; this FindCUDA build does
  not use `CMAKE_CUDA_ARCHITECTURES`.
- Include the official Thrust headers instead of forward-declaring
  `thrust::pair`, `device_allocator`, and `device_vector`, avoiding namespace
  ambiguity with CUDA 12.8 / CCCL 2.7.
- `setResolution()` updates `voxel_resolution_` as well as the CUDA core.
- ROS1 catkin-only CMake with CUDA enabled and proper library exports.

The source bundle excludes sample PCDs, benchmark applications, Python
bindings, upstream test executables, and experimental Sophus-dependent
implementations. They are not dependencies of the localization pipeline.

The CUDA translation units use the Eigen revision pinned by upstream
(`1fd5ce1002a6f30e1169b529b291216a18be2f7e`). The CUDA kNN implementation uses
the complete nine-header include closure of the upstream NVBIO revision
(`9bb7e6363c65f65e46f21df09bef98e404250f10`). Both are included under
`thirdparty/` with their license texts. The host-side PCL API uses the
distribution Eigen headers, as in the previously successful upstream build.

Exact revisions and original file hashes are recorded in
`UPSTREAM_MANIFEST.json`. This source package includes no generated build
directories, binary libraries, backup files, or Git metadata.
