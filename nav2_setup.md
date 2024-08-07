# Nav2 Tutorials
## Installation && Build
As our ROS2 is installed from the source, we can not directly install ROS2 by the `apt install` command. We must also install the Nav2 from the source and build it there. 

```
source <ros_ws>/install/setup.bash
mkdir -p ~/nav2_ws/src && cd ~/nav2_ws
git clone https://github.com/ros-planning/navigation2.git --branch main ./src/navigation2
rosdep install -r -y \
  --from-paths ./src \
  --ignore-src
colcon build \
  --symlink-install
```

After installing ROS2, we need to navigate to the ROS2 workspace `cd ~/ros2_humble`.

Running the command `git clone https://github.com/ros-planning/navigation2.git --branch $ROS_DISTRO ./src/navigation2` to clone the repository to the current workspace.

Run `rosdep install -i --from-path src --ignore-src -r -y --rosdistro humble` to install the packages.

Then build by `colcon build  --symlink-install`.
