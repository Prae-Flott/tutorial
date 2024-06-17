# Control the Create3 with Ros2 on Pi5

## Build a Node

Using actions or topics in a control node in ROS2 involves understanding the difference between these two communication methods and how to implement them in your Python script.

Remember to check the means of the actions or topics in ([https://iroboteducation.github.io/create3_docs/api/ros2/](https://iroboteducation.github.io/create3_docs/api/ros2/))

and

([https://github.com/iRobotEducation/irobot_create_msgs](https://github.com/iRobotEducation/irobot_create_msgs))

Some official tutorials are shown in this repository [https://github.com/iRobotEducation/create3_examples](https://github.com/iRobotEducation/create3_examples)

### **Topics**

Topics are a publish/subscribe communication method where nodes can publish messages to or subscribe to messages from a topic. This is often used for continuous or stateless data streams, such as sensor readings or velocity commands.

**Example: listen to a battery state from a Topic**

To get the battery state from a robot using ROS2, you need to write a subscriber node that listens to the **`/battery_state`** topic. This topic publishes messages of type **`sensor_msgs/msg/BatteryState`**. The **`BatteryState`** message type typically includes information about the battery's voltage, current, charge, capacity, and status.

We need to pay attention to the `**QoS profile**` used by iRobot. ROS ia based around sending messages to topics, and can be considered to be a message broker. Like all message brokers, there is a quality of service (QoS) that defines when a topic can be considered stale, how messages are stored if a subscriber is offline, and things like that. 

Follow the Blogs ( [https://jimbobbennett.dev/blogs/irobot-create3-subscribe-to-messages/](https://jimbobbennett.dev/blogs/irobot-create3-subscribe-to-messages/)). The relevant details are the **`QoS profile`**, particularly the **`Reliability`**, **`Durability`**, and **`Liveliness`**. To subscribe to these messages, the subscriber needs to create a **`QoSProfile`** object with the same settings, and pass this to the **`create_subscription`** call:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy

class BatteryStateSubscriber(Node):
    def __init__(self):
        super().__init__('battery_state_subscriber')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_state_callback,
            qos_profile
            )
        self.subscription  # prevent unused variable warning

    def battery_state_callback(self, msg):
        self.get_logger().info(f"Battery State: \n - Voltage: {msg.voltage} V\n - Current: {msg.current} A\n - Charge: {msg.charge} Ah\n - Capacity: {msg.capacity} Ah\n - Percentage: {msg.percentage * 100}%")

def main(args=None):
    rclpy.init(args=args)
    battery_state_subscriber = BatteryStateSubscriber()
    rclpy.spin(battery_state_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    battery_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

Then Configure the `Setup.py` file, and add the Node into the setup file:

```python
from setuptools import find_packages, setup

package_name = 'battery_state'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO: name',
    maintainer_email='TODO: email address', 
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "battery_state_subscriber = battery_state.battery_state_subscriber:main",
        ],
    },
)
```

Make sure that the module and function names in the **`entry_points`** is correct. The module **`battery_state_subscriber`** should exist in the **`battery_state`** package and it should have a **`main`** function.

### **Actions**

Actions are a more complex communication method used for pre-emptive and long-running tasks. Actions are appropriate for tasks where you need to get feedback during execution, or you might need to cancel the task.

**Example: Using an Action Client**

To use an action in a control node, you would create an action client that sends goals to an action server. Here's a basic structure for an action client:

```python
pythonCopy code
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from some_action_package.msg import SomeAction, SomeGoal

class MyActionClient(Node):
    def __init__(self):
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, SomeAction, 'some_action')

    def send_goal(self, goal_value):
        goal = SomeGoal()
        goal.value = goal_value
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # Handle feedback here
        pass

    def get_result_callback(self, future):
        result = future.result().result
        # Handle result here
        pass

def main(args=None):
    rclpy.init(args=args)
    action_client = MyActionClient()
    action_client.send_goal(some_goal_value)
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

In this example, **`SomeAction`** and **`SomeGoal`** should be replaced with the actual action types defined for your specific use case. The **`send_goal`** method sends a goal to the action server, and callbacks handle the server's response, feedback, and result.

Remember, the specifics of how you use topics or actions will depend on the capabilities of your robot and what you're trying to achieve. The ROS2 documentation and any specific documentation provided for your robot are valuable resources for understanding how to implement these communication methods effectively.

## Build a Control Node following the ros2_control standard

To achieve control function in ROS2, there is a Framework we can use called  [ros2_control](https://control.ros.org/humble/index.html#). 

When using the `ros2_control` framework, the way you interact with actions and topics can be slightly different, especially since `ros2_control` is designed to abstract hardware control through the use of controllers. The controllers provided by `ros2_control` often expose interfaces through topics or actions. Let’s explore how this would look like in a control node:

