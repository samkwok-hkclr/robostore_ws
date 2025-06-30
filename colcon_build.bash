#! /bin/bash

REALSENSE_CAMERA=realsense2_camera

REALSENSE_DESCRIPTION_MSGS_PKGS=$(colcon list -n | grep realsense2 | grep -v "^${REALSENSE_CAMERA}\$")

ROS2_CANOPEN_PKGS=$(colcon list -n | grep canopen)
LELY_CORE_PKG=lely_core_libraries

colcon build \
  --packages-select $REALSENSE_DESCRIPTION_MSGS_PKGS \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Releas

colcon build \
  --packages-select $REALSENSE_CAMERA \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_ACCELERATE_GPU_WITH_GLSL=ON

colcon build \
  --packages-ignore $REALSENSE_DESCRIPTION_MSGS_PKGS $REALSENSE_CAMERA \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release 