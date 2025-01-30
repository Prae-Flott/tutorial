# Ubuntu24.04 on Pi5 Tut

[Rasp Pi 5 Document](https://www.raspberrypi.com/documentation/computers/getting-started.html) 

[Ubuntu24.04 Document](https://ubuntu.com/blog/tag/ubuntu-24-04-lts)

## Table of Contents
- [OS Installation](#OS-Installation)
- [Ethernet Connection & NTP Server](#Create-Ethernet-on-USB0)
- [ROS2 Jazzy Installation](#ROS2-jazzy-installation)
- [RPLidar Setup](#RPLidar-Setup)
- [Nav2 & SLAM Installation](#Nav2-SLAM-Installation)

## OS-Installation
The installation is through Imager and can be downloaded from the following [link](https://www.raspberrypi.com/software/).

Then to keep the setting identical on all our 10 robots, we need to clarify some setup before starting the SD card

After sticking the SD card to the Pi, and connecting the monitor, the keyboard, and maus, we can start setup.

- Set the hostname as `robot0X-pi5` the `X` here is the unique ID for each robot, e.g. robot00-pi5. Similarly, the Username should be `robot0X`, and the Password should be `123456`, same for all the Pis.
- To connect the robot to the local Server, we have provided WiFi for local servers:

  **WIFI name at chair ASM:** `ASM_RoboFlott`, **Password:** `roboflott123`

- For the Server edition, you shall able to do these steps through Edit Setting in the imager. 

- After installing the OS, it will be nice if you can update the time manually, with an example
```
sudo date -s '23 October 2024 12:00:00'  
```
- To enable `ssh`, we also need to run:
```
sudo apt install openssh-client openssh-server -y
sudo systemctl start ssh
sudo systemctl enable ssh
```
-Add an ssh key, to connect without passwort:
```
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
ssh-copy-id -i ~/.ssh/id_rsa.pub username@server_ip
```

- Reboot the Pi by `sudo reboot`

## Create-Ethernet-on-USB0
Follow the [official tutorial for Ubuntu](https://iroboteducation.github.io/create3_docs/setup/pi4humble/), after first time boot the rasp os, we need to establish a ethernet connection to robot through `usb0`, which is the USB-C interface.

- Edit `config.txt` and add `dtoverlay=dwc2,dr_mode=peripheral` at the end of the file.

  with the command `sudo nano /boot/firmware/config.txt`.

- Edit `cmdline.txt` to add `modules-load=dwc2,g_ether` after `rootwait`.

  with the command `sudo nano /boot/firmware/cmdline.txt`

- We also need to create a new  Netplan Configuration File:.

  create the file with `sudo nano /etc/netplan/01-netcfg.yaml`

  add information about the static IP and interface with writing the following content into the file, under `ethernets`, so that the whole should look like below:
  ```
  network:
     ethernets:
         usb0:
             dhcp4: false
             optional: true
             addresses:
                - 192.168.186.3/24
         eth0:
             dhcp4: true
             dhcp6: true
             match:
                 macaddress: d8:3a:dd:9e:d5:c8
             set-name: eth0
     version: 2
  ```
  Save the file with `Ctrl+S` and `Ctrl+X`.

- Secure Configuration File Permissions and its priority by `sudo chmod 600 /etc/netplan/01-netcfg.yaml`
  
- After editing that file, you will need to run `sudo netplan generate` followed by `sudo netplan apply` to update your network configuration

- If you met a Problem with command `sudo netplan apply` regarding `systemd-networkd.service`, you need to `enable` it first.

  With command `sudo systemctl status systemd-networkd` we can check if it was installed and if it was enabled.

  If not enabled, run

  ```
  sudo systemctl enable systemd-networkd
  sudo systemctl start systemd-networkd
  ```
  To enable.

- Reboot the Pi by `sudo reboot` after doing all this.

- check if the `usb0` show up in the command `nmcli d`.

- Check the connection by `ip addr show usb0`, you suppose to see `<BROADCAST,MULTICAST,UP,LOWER_UP>` and the IP of the Server.

## Setup NTP 

As ROS2 communication depends a lot on the synchronization of the clock, we need to provide an  NTP server to synchronize the time between the robot and Pi, without having internet. The process can be found in the following [tutorial](https://iroboteducation.github.io/create3_docs/setup/compute-ntp/)

After finishing the official tutorial, you can check the connection by `ping 192.168.186.2`

## ROS2-jazzy-installation
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

  `sudo apt install -y ros-jazzy-irobot-create-msgs`

- Install a few other packages:

  `sudo apt install -y build-essential python3-colcon-common-extensions python3-rosdep ros-jazzy-rmw-cyclonedds-cpp`

- The default middleware that ROS 2 uses is `Fast DDS`

- Set up your environment by sourcing the following file

  `source /opt/ros/jazzy/setup.bash`

## RPLidar-Setup

After connecting the RPLidar with the Pi on a USB-A through an adapter board, the light on the board should keep green, representing the connection is succeed. We can type the `usb-devices`, and see if the Lidar is recognized.

On Pi5 we need to install the ros packages for RPLidar by `sudo apt install ros-jazzy-rplidar-ros`, details of this packages can be found in the [official repository](https://github.com/Slamtec/rplidar_ros)
As they may also develop the packages for the new ROS2, could be found [here](https://github.com/Slamtec/sllidar_ros2).

As rplidar_ros running requires the read and write permissions of the serial device. You can manually modify it with the following command:

`sudo chmod 777 /dev/ttyUSB0`

But a better way is to create a udev rule:


## Nav2-SLAM-Installation






   
