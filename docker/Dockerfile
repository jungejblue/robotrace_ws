# x86_64 workstation image. Jetson requires a separate JetPack/L4T ARM64 image.
ARG CUDA_BASE=nvidia/cuda:12.8.1-devel-ubuntu20.04
FROM ${CUDA_BASE}

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 ROS_DISTRO=noetic ROS_VERSION=1 ROS_PYTHON_VERSION=3
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg build-essential gcc-10 g++-10 cmake git pkg-config \
      libeigen3-dev libpcl-dev libboost-all-dev python3 python3-numpy python3-yaml \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo 'deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main' \
       > /etc/apt/sources.list.d/ros1.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-ros-base ros-noetic-pcl-ros ros-noetic-pcl-conversions \
      ros-noetic-eigen-conversions ros-noetic-tf ros-noetic-tf2-ros \
      ros-noetic-tf2-geometry-msgs ros-noetic-message-filters \
      ros-noetic-ublox-msgs ros-noetic-diagnostic-msgs ros-noetic-rosbag \
      ros-noetic-std-srvs \
    && rm -rf /var/lib/apt/lists/*

# No Livox driver/SDK: this workspace consumes Ouster PointCloud2 and Imu directly.
COPY entrypoint.sh /robotrace_entrypoint.sh
RUN chmod +x /robotrace_entrypoint.sh
WORKDIR /ws
ENTRYPOINT ["/robotrace_entrypoint.sh"]
CMD ["bash", "--rcfile", "/ws/docker/bashrc"]
