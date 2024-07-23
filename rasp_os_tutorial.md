# Raspberry Pi OS Tut

[Rasp Pi 5 Document](https://www.raspberrypi.com/documentation/computers/getting-started.html) 

[Rasp OS Bookworm Document](https://www.raspberrypi.com/documentation/computers/os.html)

## Installation
The installation is through Imager and can be downloaded from the following [link](https://www.raspberrypi.com/software/).

Choose the OS version `Raspberry Pi OS (64-bits)`, as the following image:

![Rasp1](Tutorials/rasp1.png)

Then to keep the setting identical on all our 10 robots, we need to clarify some setup before start writing in to SD card

- Open Edit Setting and set the hostname as `raspberrypi5-0x` the `x` here is the unique ID for each robot, e.g. raspberrypi5-04. Similarly, the Username should be `robot-0x`, and the Password should be `x`, a single number between 0-10.
- To connect the robot to the local Server, we have provided WiFi for local servers:

  **WIFI name at chair ASM:** `ASM_RoboFlott`, **Password:** `roboflott123`

## Create Ethernet through usb0
Follow the [official tutorial for Ubuntu](https://iroboteducation.github.io/create3_docs/setup/pi4humble/), after first time boot the rasp os, we need to establish a ethernet connection to robot through `usb0`, which is the USB-C interface.

- In the system-boot partition, edit `config.txt` and add `dtoverlay=dwc2,dr_mode=peripheral` at the end of the file.

  with the command `sudo nano /etc/boot/firmware/config.txt`.

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

   
