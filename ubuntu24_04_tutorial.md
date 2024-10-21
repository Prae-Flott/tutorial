# Ubuntu24.04 Tut

[Rasp Pi 5 Document](https://www.raspberrypi.com/documentation/computers/getting-started.html) 

[Ubuntu24.04 Document](https://ubuntu.com/blog/tag/ubuntu-24-04-lts)

## OS Installation
The installation is through Imager and can be downloaded from the following [link](https://www.raspberrypi.com/software/).

Then to keep the setting identical on all our 10 robots, we need to clarify some setup before start writing in to SD card

- Open **Edit Setting** and set the hostname as `robot0X-pi5` the `X` here is the unique ID for each robot, e.g. robot00-pi5. Similarly, the Username should be `robot0X`, and the Password should be `123456`, same for all the Pis.
- To connect the robot to the local Server, we have provided WiFi for local servers:

  **WIFI name at chair ASM:** `ASM_RoboFlott`, **Password:** `roboflott123`

## ROS2 jazzy installation
From the [iRobot official tutorial](https://iroboteducation.github.io/create3_docs/setup/ubuntu2204/) and the [ROS2 jazzy documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), we sum the installation into following steps.

- Make sure you have a locale that supports UTF-8.

  `locale  # check for UTF-8`
  
- Ensure that the Ubuntu Universe repository is enabled by checking the output of this command:

  `apt-cache policy | grep universe`
  
- Now add the ROS 2 GPG key with apt.

  `sudo apt update && sudo apt install curl -y`

  `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`

- Then add the repository to your sources list.

  ```
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ```

- Make sure your other packages are up to date

  `sudo apt update && sudo apt upgrade`

- ROS-Base Install (Bare Bones):

  `sudo apt install ros-jazzy-ros-base`

  or if the tutorial or Rviz is required:

  `sudo apt install ros-jazzy-desktop`

- Next add the Create® 3 messages:

  `sudo apt install -y ros-humble-irobot-create-msgs`

- Install a few other packages:

  `sudo apt install -y build-essential python3-colcon-common-extensions python3-rosdep ros-jazzy-rmw-cyclonedds-cpp`

- The default middleware that ROS 2 uses is `Fast DDS`

- Set up your environment by sourcing the following file

  `source /opt/ros/jazzy/setup.bash`


## Create Ethernet through usb0
Follow the [official tutorial for Ubuntu](https://iroboteducation.github.io/create3_docs/setup/pi4humble/), after first time boot the rasp os, we need to establish a ethernet connection to robot through `usb0`, which is the USB-C interface.

- In the system-boot partition, edit `config.txt` and add `dtoverlay=dwc2,dr_mode=peripheral` at the end of the file.

  with the command `sudo nano /boot/firmware/config.txt`.

- In the system-boot partition, edit `cmdline.txt` to add `modules-load=dwc2,g_ether` after `rootwait`.

  with the command `sudo nano /boot/firmware/cmdline.txt`

- In the system-boot partition, we need to setup a network configuration without using network manager.

  create a new file with `sudo nano /etc/network/interfaces.d/g_ether`

  add information about the static IP and interface with writing the following content into the file:

  ```
  auto usb0
  allow-hotplug usb0
  iface usb0 inet static
        address 192.168.186.3/24
        netmask 255.255.0.0

  auto usb0.1
  allow-hotplug usb0.1
  iface usb0.1 inet dhcp


  EOF
  ```
- Reboot the Pi by `sudo reboot`

- Check the connection by `ip addr show usb0`, you suppose to see `<BROADCAST,MULTICAST,UP,LOWER_UP>` and the IP of the Server.

## Setup NTP 

As ROS2 communication depends a lot on the synchronization of the clock, we need to provide an  NTP server to synchronize the time between the robot and Pi, without having internet. The process can be found in the following [tutorial](https://iroboteducation.github.io/create3_docs/setup/compute-ntp/)

   
