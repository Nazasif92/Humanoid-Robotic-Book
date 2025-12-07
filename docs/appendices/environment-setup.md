---
sidebar_label: "Appendix A: Environment Setup"
---

# Appendix A: Environment Setup

This appendix provides detailed instructions for setting up your development environment to work with the examples in this book.

## Prerequisites

Before starting, ensure you have the following installed:

- Ubuntu 22.04 LTS (or use WSL2 on Windows)
- ROS 2 Humble Hawksbill
- Python 3.10+
- Git
- Docker (optional but recommended)

## Installing ROS 2 Humble

1. Set locale:
```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

2. Add ROS 2 apt repository:
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-keyring.gpg | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

3. Install ROS 2 packages:
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

4. Install colcon build system:
```bash
sudo apt install python3-colcon-common-extensions
```

## Setting up the Environment

Add the following to your `~/.bashrc` file:
```bash
source /opt/ros/humble/setup.bash
```

Then reload your environment:
```bash
source ~/.bashrc
```

## Testing the Installation

Verify ROS 2 is working:
```bash
ros2 topic list
```

This should not produce any errors.

## Docker Setup (Optional)

For a consistent development environment, you can use the official ROS 2 Docker image:

```bash
docker pull ros:humble
docker run -it ros:humble bash
source /opt/ros/humble/setup.bash
```