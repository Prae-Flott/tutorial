# iRobot Create 3

The Basic Hardware and Software information are shown on the [Documentation Website](https://iroboteducation.github.io/create3_docs/).

And the original [Create 3 Robot Project Code](https://github.com/iRobotEducation/irobot-edu-python-sdk/tree/main/examples/root_robots) is stored in Github. There are also some examples there.

The first step is to connect the robot to the WLAN network and update the firmware. To do that, we need to follow the [setup guidance](https://edu.irobot.com/create3-setup).

It is also possible to play around with iRobot Create 3 with an online [Coding Platform](https://python.irobot.com/) using Python.

# Create3 with Rasp Pi

[https://edu.irobot.com/learning-library/connect-create-3-to-raspberry-pi](https://edu.irobot.com/learning-library/connect-create-3-to-raspberry-pi)

# Set up Ubuntu and ROS2 on Pi5

**Dockerfile for building ROS2 Galactic image**

[Dockerfile](Tutorials/Dockerfile.txt)

**Installation** 

- network-manager
- `apt install iputils-ping`
- install nmap for showing devices in network:

sudo apt update
sudo apt install nmap

sudo nmap -sn 192.168.1.0/24

- Install iproute2 to show own ip Adress:
sudo apt install iproute2
`ip addr show`

# Set up Rasp OS and ROS2 on Pi5

So far the Pasp OS provided on Pi5 is based on [Debian 12 Bookworm](https://www.debian.org/News/2023/20230610). 

Installation of  the PaspOS can be easily done with the help of the [official Imager](https://www.raspberrypi.com/software/) 

## ROS2 on Debian 12

The Debian 12 could also been supported by ROS2, if we check the original package code of ROS2 from ([http://packages.ros.org/ros2/ubuntu/dists](http://packages.ros.org/ros2/ubuntu/dists)). In which we can find the folder for Bookworm. So it should be possible to install on a Bookworm system.

The ROS2 version we used here is the [Humble](https://docs.ros.org/en/humble/index.html#), which also been introduced in the guidence of Create 3.

### Installation

As Debian Linux only got Tier3 supported by ROS2. We are not able to install it via `apt` command, we could follow the step of install from source:

[Ubuntu (source) — ROS 2 Documentation: Humble  documentation](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

**Attention! We need to modify at some place**

At the step adding the ROS 2 apt repository to your system remove the universe step, it do not work for a debian os.

```bash
sudo apt install software-properties-common
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to the source list

But we need to change the get OS name command in Ubuntu, fit it to Debian system, like `$(. /etc/os-release && echo $VERSION_CODENAME)`.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
The `VERSION_CODENAME` should be `Bookworm` for our Debian12-based Rasp OS.

[](Tutorials%208615955266424c32bb82012bc71e676a/Untitled%2036791497ed444b06b52bc59ecba2bce8.md)

## Build a subscriber Node

[https://jimbobbennett.dev/blogs/irobot-create3-subscribe-to-messages/](https://jimbobbennett.dev/blogs/irobot-create3-subscribe-to-messages/)

# Use QEMU to Run original image

Follow this instruction:

[https://hub.docker.com/r/tonistiigi/binfmt?source=post_page-----208929004510--------------------------------](https://hub.docker.com/r/tonistiigi/binfmt?source=post_page-----208929004510--------------------------------)

