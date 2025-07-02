# RoboStore

## Prerequisites

* CAN Module w/ Linux SocketCAN installed.
* TBC...

## Installation

### 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/samkwok-hkclr/robostore_ws
```

### 2. Install the Dependencies

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y
```

### 3. Build the packages

```bash
colcon build
```

### 4. TBD

```bash
# to be done
```