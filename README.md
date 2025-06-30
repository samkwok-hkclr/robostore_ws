# RoboStore

## Prerequisites

* A host PC with Docker and Git installed.
* CAN Module installed.

## Installation

### 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/samkwok-hkclr/robostore_ws
```

### 2. Install the Dependencies

```bash
sudo rosdep init
rosdep update
sudo rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y
```