# ROS 2 Topics, Publisher, and Subscriber Nodes

## Introduction: Topics and Pub/Sub Model

In ROS 2, nodes communicate using the publish/subscribe (pub/sub) model via **topics**. Publishers send messages to a topic, and subscribers receive messages from that topic. This decouples nodes and enables flexible communication.

---

## Classic Example: Talker and Listener Nodes

The "talker" and "listener" example is the simplest demonstration of the pub/sub model.

### 1. Start the Talker Node

The talker node publishes messages to the `/chatter` topic.

```bash
ros2 run demo_nodes_py talker
```

### 2. Start the Listener Node

The listener node subscribes to the `/chatter` topic and prints received messages.

```bash
ros2 run demo_nodes_py listener
```

### 3. Inspect Topics

- List active topics:
  ```bash
  ros2 topic list
  ```
- Echo messages on `/chatter`:
  ```bash
  ros2 topic echo /chatter
  ```

---

## Turtlesim: Visual Topic Demo

`turtlesim` is a graphical tool to visualize topics and pub/sub in action.

### 1. Start turtlesim_node

```bash
ros2 run turtlesim turtlesim_node
```

### 2. Control the Turtle with teleop

```bash
ros2 run turtlesim turtle_teleop_key
```

- The teleop node publishes velocity commands to `/turtle1/cmd_vel`.
- The turtlesim node subscribes to `/turtle1/cmd_vel` and moves the turtle.

### 3. Inspect turtlesim topics

- List topics:
  ```bash
  ros2 topic list
  ```
- Echo velocity commands:
  ```bash
  ros2 topic echo /turtle1/cmd_vel
  ```

### 4. Topic Info and Message Type

To get information about a topic and its message type:

```bash
ros2 topic info /turtle1/cmd_vel
```
Example output:
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

To see the structure of the message type:

```bash
ros2 interface show geometry_msgs/msg/Twist
```
This expresses velocity in free space broken into its linear and angular parts:
```
# This expresses velocity in free space broken into its linear and angular parts.

Vector3 linear
  float64 x
  float64 y
  float64 z

Vector3 angular
  float64 x
  float64 y
  float64 z
```

---

## Example: Publisher Node for Turtlesim

To publish velocity commands to turtlesim, you need these dependencies in your package.xml:

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>turtlesim</depend>
```

Here is a minimal publisher node that makes the turtle move in a circle by publishing `Twist` messages to `/turtle1/cmd_vel`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

- The publisher is created for the `/turtle1/cmd_vel` topic using the `Twist` message type.
- The timer periodically calls `send_velocity_command`, which publishes a velocity command to move the turtle in a circle.

---

## Example: Subscriber Node for Turtlesim Pose

To subscribe to the turtle's pose, use the following dependencies in your package.xml:

```xml
<depend>rclpy</depend>
<depend>turtlesim</depend>
```

The `/turtle1/pose` topic uses the `turtlesim/msg/Pose` message type. You can inspect it with:

```bash
ros2 topic info /turtle1/pose
ros2 interface show turtlesim/msg/Pose
```
Message structure:
```
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

Here is a minimal subscriber node for `/turtle1/pose`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.subscription = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Pose subscriber node has been started")

    def pose_callback(self, msg):
        self.get_logger().info(
            f"x: {msg.x}, y: {msg.y}, theta: {msg.theta}, "
            f"linear_velocity: {msg.linear_velocity}, angular_velocity: {msg.angular_velocity}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

- The subscriber listens to `/turtle1/pose` and prints the turtle's position and velocity.

---

## Running Publisher and Subscriber Nodes
###->>> Dont forget to add the nodes as excutables <<<-

1. Build your workspace:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

2. Run the publisher node:
   ```bash
   ros2 run my_first_py_pkg draw_circle
   ```

3. Run the subscriber node (in a separate terminal):
   ```bash
   ros2 run my_first_py_pkg pose_subscriber
   ```

---

## Visualizing Topics

- Use `rqt_graph` to visualize nodes and topics.
- List active topics:
  ```bash
  ros2 topic list
  ```
- Echo messages on a topic:
  ```bash
  ros2 topic echo /topic
  ```

---

## Reference

- See [`01/Readme.md`](../01/Readme.md) for workspace and package setup.

---

This guide explains how to use topics and create publisher/subscriber nodes in ROS 2, building on the basics from section 01. It now includes classic talker/listener and turtlesim examples, with topic info and message type inspection, and practical publisher and subscriber nodes for turtlesim using `geometry_msgs/msg/Twist` and `turtlesim/msg/Pose`.
