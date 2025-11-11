#!/bin/bash

REALSENSE_CAMERA=realsense2_camera

REALSENSE_DESCRIPTION_MSGS_PKGS=$(colcon list -n | grep realsense2 | grep -v "^${REALSENSE_CAMERA}\$")

colcon build \
  --packages-select $REALSENSE_DESCRIPTION_MSGS_PKGS \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release

colcon build \
  --packages-select $REALSENSE_CAMERA \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSE_LIFECYCLE_NODE=ON \
  # -DBUILD_ACCELERATE_GPU_WITH_GLSL=ON

colcon build \
  --packages-ignore $REALSENSE_DESCRIPTION_MSGS_PKGS $REALSENSE_CAMERA vision_recognition \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release 