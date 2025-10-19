# Lab 04: Keyboard Teleop Control for Warehouse Robot

**Course:** Robotics Lab
**Department:** Computer Science, FCIS Mansoura University
**Semester:** Fall 2025
**ROS Version:** ROS 2 Jazzy Jalisco
**Gazebo Version:** Gazebo Jetty (gz-sim)

---

## Overview

In this lab, you'll create a custom keyboard teleop node to manually control your warehouse robot. This builds upon Lab 03's warehouse simulation by adding intuitive keyboard control.

**Learning Objectives:**
- Create a custom ROS 2 Python node for keyboard input
- Understand velocity control and publishing to cmd_vel topic
- Implement safety features and emergency stops
- Test and verify robot control in simulation

**Prerequisites:**
- Completed Lab 03 (Warehouse Simulation)
- Working warehouse_simulation package
- Robot responding correctly to /cmd_vel messages

---

## Part 1: Understanding Keyboard Teleop

### What is Teleop?

**Teleoperation (Teleop)** = Remote operation of a robot by a human operator.

For our warehouse robot:
- **Input:** Keyboard presses (w, a, s, d, etc.)
- **Processing:** ROS node converts keypresses to velocity commands
- **Output:** Twist messages published to `/cmd_vel` topic
- **Result:** Robot moves in Gazebo

### Why Custom Teleop?

Standard ROS teleop packages exist, but creating our own allows:
1. **Customization:** Tailor controls to our specific needs
2. **Learning:** Understand how ROS nodes work
3. **Features:** Add custom functionality (speed limits, emergency stop, etc.)
4. **Integration:** Better integration with our warehouse_simulation package

---

## Part 2: Teleop Node Design

### Control Scheme

We'll use **incremental velocity control**:

```
w : Increase forward speed (+0.1 m/s per press)
x : Increase backward speed (-0.1 m/s per press)
a : Increase left turn rate (+0.2 rad/s per press)
d : Increase right turn rate (-0.2 rad/s per press)
s : Stop (set all velocities to 0)
SPACE : Emergency stop (same as 's')
q : Quit program
```

**Why incremental?**
- Smooth acceleration/deceleration
- Precise control over robot speed
- Robot maintains velocity until you change it

### Velocity Limits

For safety, we'll implement limits:
- **Max linear velocity:** 1.0 m/s
- **Max angular velocity:** 2.0 rad/s

This prevents the robot from moving dangerously fast.

---

## Part 3: Implementation

### Step 3.1: Create the Teleop Node

Create the file:
```bash
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws/warehouse_simulation/warehouse_simulation
nano teleop_key.py
```

Add the complete teleop node code:

```python
#!/usr/bin/env python3
"""
Teleop Key Node for Warehouse Robot
Control the robot using keyboard keys:
    w/x : increase/decrease linear velocity
    a/d : increase/decrease angular velocity
    s   : stop
    SPACE : emergency stop
    q   : quit
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopKeyNode(Node):
    def __init__(self):
        super().__init__('teleop_key')

        # Publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Velocity settings
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.linear_step = 0.1  # m/s
        self.angular_step = 0.2  # rad/s
        self.max_linear = 1.0
        self.max_angular = 2.0

        self.get_logger().info('Teleop Key Node Started!')
        self.get_logger().info('Use w/x for linear, a/d for angular, s to stop, SPACE for emergency stop, q to quit')

    def get_key(self):
        """Get a single keypress from the terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def update_velocity(self, linear_change, angular_change):
        """Update velocity with limits"""
        self.linear_velocity += linear_change
        self.angular_velocity += angular_change

        # Apply limits
        self.linear_velocity = max(-self.max_linear, min(self.max_linear, self.linear_velocity))
        self.angular_velocity = max(-self.max_angular, min(self.max_angular, self.angular_velocity))

    def publish_velocity(self):
        """Publish the current velocity"""
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher.publish(msg)

        self.get_logger().info(f'Linear: {self.linear_velocity:.2f} m/s, Angular: {self.angular_velocity:.2f} rad/s')

    def run(self):
        """Main control loop"""
        print("\n" + "="*60)
        print("Warehouse Robot Teleop Control")
        print("="*60)
        print("Controls:")
        print("  w : Increase forward speed")
        print("  x : Increase backward speed")
        print("  a : Turn left (increase)")
        print("  d : Turn right (increase)")
        print("  s : Stop all movement")
        print("  SPACE : Emergency stop")
        print("  q : Quit")
        print("="*60 + "\n")

        try:
            while rclpy.ok():
                key = self.get_key()

                if key == 'w':
                    self.update_velocity(self.linear_step, 0)
                    self.publish_velocity()

                elif key == 'x':
                    self.update_velocity(-self.linear_step, 0)
                    self.publish_velocity()

                elif key == 'a':
                    self.update_velocity(0, self.angular_step)
                    self.publish_velocity()

                elif key == 'd':
                    self.update_velocity(0, -self.angular_step)
                    self.publish_velocity()

                elif key == 's' or key == ' ':
                    self.linear_velocity = 0.0
                    self.angular_velocity = 0.0
                    self.publish_velocity()
                    if key == ' ':
                        self.get_logger().warn('EMERGENCY STOP!')

                elif key == 'q':
                    self.get_logger().info('Quitting...')
                    break

                elif key == '\x03':  # Ctrl+C
                    break

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        finally:
            # Stop the robot before exiting
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.publish_velocity()
            self.get_logger().info('Teleop Key Node Stopped')


def main(args=None):
    rclpy.init(args=args)

    teleop_node = TeleopKeyNode()

    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

---

### Step 3.2: Make the File Executable

```bash
chmod +x ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws/warehouse_simulation/warehouse_simulation/teleop_key.py
```

---

### Step 3.3: Update setup.py

Edit the setup.py file:
```bash
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws/warehouse_simulation
nano setup.py
```

Update the `entry_points` section:

```python
entry_points={
    'console_scripts': [
        'teleop_key = warehouse_simulation.teleop_key:main',
    ],
},
```

Save and exit.

---

### Step 3.4: Build the Package

```bash
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws
colcon build --packages-select warehouse_simulation
source install/setup.bash
```

**Expected output:**
```
Starting >>> warehouse_simulation
Finished <<< warehouse_simulation [X.XXs]

Summary: 1 package finished [X.XXs]
```

---

## Part 4: Testing the Teleop Node

### Terminal 1: Launch the Simulation

```bash
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws
source install/setup.bash
ros2 launch warehouse_simulation warehouse_simulation.launch.py
```

**Wait for:**
- Gazebo window to open
- Robot to appear in the warehouse
- Bridge messages showing topic connections

---

### Terminal 2: Run the Teleop Node

```bash
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws
source install/setup.bash
ros2 run warehouse_simulation teleop_key
```

**You should see:**
```
============================================================
Warehouse Robot Teleop Control
============================================================
Controls:
  w : Increase forward speed
  x : Increase backward speed
  a : Turn left (increase)
  d : Turn right (increase)
  s : Stop all movement
  SPACE : Emergency stop
  q : Quit
============================================================

[INFO] [teleop_key]: Teleop Key Node Started!
[INFO] [teleop_key]: Use w/x for linear, a/d for angular, s to stop, SPACE for emergency stop, q to quit
```

---

### Test Sequence

Try this sequence to test all controls:

1. **Press `w` 3 times:**
   - Velocity should increase: 0.1, 0.2, 0.3 m/s
   - Robot should move forward in Gazebo

2. **Press `a` 2 times:**
   - Angular velocity should increase: 0.2, 0.4 rad/s
   - Robot should start turning left while moving forward

3. **Press `s`:**
   - Both velocities should drop to 0
   - Robot should stop

4. **Press `x` 2 times:**
   - Linear velocity should become: -0.1, -0.2 m/s
   - Robot should move backward

5. **Press `d` 3 times:**
   - Angular velocity should become: -0.2, -0.4, -0.6 rad/s
   - Robot should turn right while moving backward

6. **Press `SPACE`:**
   - Emergency stop message should appear
   - Robot should stop immediately

7. **Press `q`:**
   - Program should exit cleanly

---

## Part 5: Understanding the Code

### Key Concepts

#### 1. ROS Node Structure

```python
class TeleopKeyNode(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
```

- Inherits from `Node` class
- Creates publisher for Twist messages on `/cmd_vel` topic

#### 2. Terminal Input Handling

```python
def get_key(self):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
```

- Sets terminal to raw mode to read single keypresses
- Restores terminal settings after reading

#### 3. Incremental Velocity Control

```python
def update_velocity(self, linear_change, angular_change):
    self.linear_velocity += linear_change
    self.angular_velocity += angular_change

    # Apply limits
    self.linear_velocity = max(-self.max_linear, min(self.max_linear, self.linear_velocity))
```

- Adds change to current velocity (incremental)
- Clamps to max/min limits

#### 4. Publishing Twist Messages

```python
def publish_velocity(self):
    msg = Twist()
    msg.linear.x = self.linear_velocity
    msg.angular.z = self.angular_velocity
    self.publisher.publish(msg)
```

- Creates Twist message
- Sets linear.x (forward/backward) and angular.z (rotation)
- Publishes to `/cmd_vel`

---

## Part 6: Monitoring and Debugging

### Monitor Published Messages

Open a new terminal:
```bash
source ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws/install/setup.bash
ros2 topic echo /cmd_vel
```

As you press keys, you'll see:
```yaml
linear:
  x: 0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2
---
```

### Check Node Information

```bash
ros2 node list
# Should show: /teleop_key

ros2 node info /teleop_key
# Shows publishers, subscribers, etc.
```

### Visualize the System

```bash
ros2 run rqt_graph rqt_graph
```

You should see:
```
/teleop_key --> /cmd_vel --> /parameter_bridge --> Gazebo
```

---

## Part 7: Enhancements and Exercises

### Exercise 1: Add Speed Presets

Modify the code to add number keys (1-5) for preset speeds:

```python
# In the run() method, add:
elif key == '1':
    self.linear_step = 0.05
    self.get_logger().info('Speed: SLOW')
elif key == '2':
    self.linear_step = 0.1
    self.get_logger().info('Speed: NORMAL')
elif key == '3':
    self.linear_step = 0.2
    self.get_logger().info('Speed: FAST')
```

### Exercise 2: Add Position Display

Print current position from odometry:

```python
# Add subscriber in __init__:
self.odom_sub = self.create_subscription(
    Odometry, '/odom', self.odom_callback, 10
)

# Add callback:
def odom_callback(self, msg):
    self.position = msg.pose.pose.position
```

### Exercise 3: Collision Warning

Add warning when obstacles detected:

```python
# Add subscriber for laser scan:
self.scan_sub = self.create_subscription(
    LaserScan, '/scan', self.scan_callback, 10
)

def scan_callback(self, msg):
    min_dist = min(msg.ranges)
    if min_dist < 0.5:  # 50cm
        self.get_logger().warn(f'Obstacle detected at {min_dist:.2f}m!')
```

---

## Part 8: Alternative Teleop Options

### Option 1: Standard ROS Teleop (Hold-to-Move)

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Differences:**
- Must hold key to maintain velocity
- Different key layout (i/j/k/l for movement)
- No incremental control

### Option 2: Gamepad Control

```bash
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy

# Terminal 1
ros2 run joy joy_node

# Terminal 2
ros2 run teleop_twist_joy teleop_node
```

**Requires:** USB gamepad/joystick

---

## Part 9: Troubleshooting

### Issue 1: "Inappropriate ioctl for device" Error

**Cause:** Running in non-interactive terminal (IDE/script)

**Solution:** Run teleop_key in a real terminal window

### Issue 2: Robot Doesn't Move

**Check:**
```bash
# Is simulation running?
ros2 topic list | grep cmd_vel

# Is teleop publishing?
ros2 topic echo /cmd_vel

# Are there multiple publishers?
ros2 topic info /cmd_vel
```

**Solution:** Ensure only one publisher to /cmd_vel

### Issue 3: Robot Moves on Its Own

**Check:**
```bash
ros2 topic echo /cmd_vel
```

**Solution:** Something else is publishing to /cmd_vel. Stop other nodes.

### Issue 4: Keys Not Responding

**Check:** Terminal window has focus (is active)

**Solution:** Click on the terminal running teleop_key before pressing keys

---

## Submission Requirements

### What to Submit

1. **Code Package:**
   ```bash
   cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws
   tar -czf lab04_teleop.tar.gz warehouse_simulation/
   ```

2. **Video Demonstration** (2-3 minutes):
   - Show teleop control interface
   - Demonstrate all movement commands (w, x, a, d, s)
   - Show robot moving in Gazebo in response to keys
   - Show velocity feedback in terminal
   - Demonstrate emergency stop (SPACE)

3. **Lab Report** (PDF format):
   - Introduction
   - Explain how the teleop node works
   - Screenshots showing:
     * Teleop running with control messages
     * Robot moving in Gazebo
     * Topic echo output
     * Node graph (rqt_graph)
   - Answer discussion questions
   - Describe one enhancement you implemented (optional)
   - Conclusion

---

## Discussion Questions

Answer these in your report:

1. **Explain the difference between incremental velocity control (our teleop) and direct velocity control (standard teleop). What are the advantages of each?**

2. **Why do we need to clamp velocities to max/min limits? What could happen if we didn't?**

3. **The `get_key()` function uses `tty.setraw()` and then restores settings. Why is this necessary?**

4. **In the Twist message, we set `linear.x` and `angular.z`. Why don't we use linear.y or angular.x/y for a ground robot?**

5. **How would you modify the teleop node to make the robot follow a predefined path (like a square) automatically?**

6. **What happens if the teleop node crashes while the robot is moving? How could you implement a safety feature to handle this?**

---

## Grading Rubric

| Component | Points | Criteria |
|-----------|--------|----------|
| **Teleop Implementation** | 30 | Node works correctly, all keys functional |
| **Code Quality** | 20 | Clean, well-commented, follows best practices |
| **Testing** | 20 | Thorough testing demonstrated |
| **Video Demonstration** | 15 | Clear, shows all features |
| **Lab Report** | 15 | Well-written, answers all questions |
| **Total** | **100** | |

---

## Bonus Challenges (+10 points each, max +30)

1. **Multi-Speed Control:** Implement 5 speed levels (keys 1-5) with visual feedback

2. **Smooth Ramping:** Add gradual acceleration/deceleration instead of instant velocity changes

3. **Path Recording:** Record robot path and save to file for later playback

4. **Safety Zone:** Auto-stop if obstacle detected within 30cm

5. **Status Dashboard:** Print real-time dashboard showing velocity, position, and nearby obstacles

---

## Quick Reference

### File Structure

```
warehouse_simulation/
├── warehouse_simulation/
│   ├── __init__.py
│   └── teleop_key.py          ← Created in this lab
├── config/
│   └── bridge.yaml
├── launch/
│   └── warehouse_simulation.launch.py
├── package.xml
└── setup.py                   ← Updated in this lab
```

### Key Commands

```bash
# Build
colcon build --packages-select warehouse_simulation

# Launch simulation (Terminal 1)
ros2 launch warehouse_simulation warehouse_simulation.launch.py

# Run teleop (Terminal 2)
ros2 run warehouse_simulation teleop_key

# Monitor velocity (Terminal 3)
ros2 topic echo /cmd_vel

# Check nodes
ros2 node list
ros2 node info /teleop_key

# Visualize
ros2 run rqt_graph rqt_graph
```

### Controls Summary

```
w     - Forward speed +0.1 m/s
x     - Backward speed -0.1 m/s
a     - Turn left +0.2 rad/s
d     - Turn right -0.2 rad/s
s     - Stop
SPACE - Emergency stop
q     - Quit
```

---

**Happy teleoperating! Enjoy controlling your warehouse robot! 🎮🤖**

---

*Lab content prepared for FCIS Mansoura University - Fall 2025*
*Updated for Gazebo Jetty and ROS 2 Jazzy*
