# RoboStore Workspace

## Prerequisites

* ROS2 Humble
* TBC...

## Installation

### 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/samkwok-hkclr/robostore_ws
```

### 2. Install the Dependencies

```bash
# ROS_PYTHON_VERSION=3
# ROS_DISTRO=humble
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y
```

### 3. Build the packages

```bash
./colcon_build.bash
```

### 4. TBD

```bash
# to be done

```
