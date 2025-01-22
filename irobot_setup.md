# iRobot Create 3
(Some important/ interest Links)

The Basic Hardware and Software information are shown on the [Documentation Website](https://iroboteducation.github.io/create3_docs/).

And the original [Create 3 Robot Project Code](https://github.com/iRobotEducation/irobot-edu-python-sdk/tree/main/examples/root_robots) is stored in Github. There are also some examples there.

Playing around with iRobot Create 3 with an online [Coding Platform](https://python.irobot.com/) using Python is also possible.

A Tutorial contains the whole process is ([https://jimbobbennett.dev/blogs/irobot-create3-connect-a-pi/](https://jimbobbennett.dev/blogs/irobot-create3-connect-a-pi/))

# WIFI Connection and Basic Configuration

The first step after getting the robot is to update the firmware. To do that, follow the [setup guidance](https://edu.irobot.com/create3-setup).

After rebooting the robot, connect again to the wifi of the robot and go into the main page (`192.168.10.1`) again.

Here we need to do some configurations as follows:

### Firmware Update
- The first step after getting a new Create3 Robot is to update its firmware to the new updated `I.0.0 FastDDS` which is introduced in [docs pages](https://iroboteducation.github.io/create3_docs/releases/i_0_0/)
  This new version of firmware works for Ros2 Iron but is also paired with Ros2 Fuzzy, which we used on Ubuntu 24.04LTS.

### Connect
- Open from the top menu `Connect`
- The Hostname should be given in a form like `robot-0n` (n is 1-10).
- Bluetooth name should be same as hostname.
- Press `update`


### Configurations
- Open from the top menu `Application` -> `Configurations`
- Select the `fastrtps` as our RMW (Ros Middle Ware),  as the [official document](https://iroboteducation.github.io/create3_docs/setup/xml-config/), RMW should be identical on both the Robot and Pi5.
- Save the changes

### NTP configurations
Follow the [official tutorial](https://iroboteducation.github.io/create3_docs/setup/compute-ntp/), we have the following steps for ethernet connection through usb-c
- Open from the top menu `Beta features` -> `ntp.conf`
- Add the following if it does not exist: `server 192.168.186.3 iburst`
- Save the change
- reboot the robot through `Application` -> `Reboot Robot`.

**remember to switch the bottom on the [Adapter board](https://iroboteducation.github.io/create3_docs/hw/adapter/) to the USB side.** 

<strike>

### Namespace

To face the problem with multiple robots, we need to define the `namespace` for each robot.

After connecting with the robot WIFI( wifi name should look like CREATE-XXXX), open robot page `192.168.10.1` you can find the namespace slot under Configuration.

The namespace should be given in a form like `/rn` (n is 1-10), n is also the number for the username of the Raspberry Pi, connected to the robot.

</strike>

## Others

If it comes up with a Problem when you test with `ros2 action list` try to connect the robot to its wifi and **reboot the Robot, it does help a lot!**

![Untitled](Tutorials/Untitled.png)


# Create3 with Rasp Pi

[https://edu.irobot.com/learning-library/connect-create-3-to-raspberry-pi](https://edu.irobot.com/learning-library/connect-create-3-to-raspberry-pi)


# Use QEMU to Run original image

Follow this instruction:

[https://hub.docker.com/r/tonistiigi/binfmt?source=post_page-----208929004510--------------------------------](https://hub.docker.com/r/tonistiigi/binfmt?source=post_page-----208929004510--------------------------------)

