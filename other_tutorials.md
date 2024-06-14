# Tutorials

# Useful Links for Tutorials

[First Look: MicroROS-Pi5 ROS2 Robot Car | RobotShop Community](https://community.robotshop.com/robots/show/microros-pi5-ros2-robot-car)

- [https://github.com/iRobotEducation/create3_docs/discussions/443](https://github.com/iRobotEducation/create3_docs/discussions/443): :
    
    I just did a 100% Pi5 load test powered from the Create3 USB-C port (no USB peripherals powered by the Pi5) NO PROBLEM! Full Speed, Full Load, No Throttling - 12.7W total
    
- [https://core-electronics.com.au/guides/oak-d-lite-raspberry-pi/](https://core-electronics.com.au/guides/oak-d-lite-raspberry-pi/): RGB Single Camera and Stereo Camera in one, can be used for AI Taks

![Smorgasboard.png](Tutorials%208615955266424c32bb82012bc71e676a/Smorgasboard.png)

- [https://github.com/iRobotEducation/create3_docs/discussions?discussions_q=pi5](https://github.com/iRobotEducation/create3_docs/discussions?discussions_q=pi5)
- [https://github.com/UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)


# Create a map for Create3 with Nav2

[https://www.youtube.com/watch?v=BkRVhxYEmGU](https://www.youtube.com/watch?v=BkRVhxYEmGU)

To generate a map we could use the [Nav2](https://navigation.ros.org/) package, paired with ROS2

Based on the sensor on Create3 (7 IR Sensor), it is difficult to map the real world, according to an answer from an iRobot staff ([https://discourse.ros.org/t/irobot-create-3-with-nav2/34025/1](https://discourse.ros.org/t/irobot-create-3-with-nav2/34025/1)). 

So, for future steps, we may need a Lidar or an RGB-D camera for mapping the environment for the robot. Please have a look at the [official example](https://github.com/iRobotEducation/create3_examples/tree/galactic/create3_lidar) The odometry sensor on Create 3 can be used to localize the robot.

# Turtlebot

[https://www.youtube.com/watch?v=HW8RRbBEO7Y](https://www.youtube.com/watch?v=HW8RRbBEO7Y)

Through the following tutorial [https://github.com/Tinker-Twins/Autonomy-Science-And-Systems?tab=readme-ov-file](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems?tab=readme-ov-file), we can see some similar ROS2 programming in Turtle3 Robot project. 

[TurtleBot 4 - Google Search](https://www.google.com/search?q=TurtleBot+4&oq=tur&gs_lcrp=EgZjaHJvbWUqDggAEEUYJxg7GIAEGIoFMg4IABBFGCcYOxiABBiKBTIGCAEQRRhAMgYIAhBFGDkyBggDEEUYPDIGCAQQRRg9MgYIBRBFGD0yBggGEEUYQTIGCAcQRRhB0gEIMjk4NGowajSoAgCwAgA&sourceid=chrome&ie=UTF-8)

# Simulator of Create3

iRobot also provides a [Simulator for Create 3](https://github.com/iRobotEducation/create3_sim), which needs to run with the ROS2.

Here is a tutorial about using the Turtle Robot for
