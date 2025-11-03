# Lab 05: TurtleBot3 Simulation with RViz - Camera, LiDAR & Odometry

## Overview

This lab introduces TurtleBot3 robot simulation with comprehensive sensor monitoring using RViz. You'll learn to visualize camera feeds, LiDAR data, and understand odometry concepts for robot localization.

## Learning Objectives

- Set up TurtleBot3 simulation in Gazebo
- Use RViz to monitor camera and LiDAR sensors
- Understand odometry data and robot pose estimation
- Implement teleoperation control for TurtleBot3
- Analyze sensor data for navigation applications

---

## Prerequisites

### Software Requirements
- ROS 2 Jazzy Jalisco
- TurtleBot3 packages (from source)
- Gazebo simulation environment
- RViz visualization tool

### Installation Commands

**Install Gazebo and Dependencies:**
```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic

# Install Navigation and SLAM packages
sudo apt install ros-jazzy-cartographer ros-jazzy-cartographer-ros
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-rviz2 ros-jazzy-rqt*
```

**Install TurtleBot3 Packages (in your ros_ws):**
```bash
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Build the workspace
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws
colcon build --symlink-install
source install/setup.bash
```


**Environment Setup:**
```bash
# Add to ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc
```

---

## Understanding Key Concepts

### What is Odometry?

**Odometry** is the use of data from motion sensors to estimate change in position over time. In robotics:

- **Purpose**: Track robot's position and orientation (pose) relative to a starting point
- **Data Sources**: Wheel encoders, IMU (Inertial Measurement Unit), visual odometry
- **Limitations**: Accumulates error over time (drift), affected by wheel slip, sensor noise
- **Applications**: Dead reckoning, sensor fusion with other localization methods

### Odometry Message Structure

The `nav_msgs/msg/Odometry` message contains:
```
# Pose (position + orientation) with uncertainty
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position      # x, y, z coordinates
    geometry_msgs/Quaternion orientation  # rotation as quaternion
  float64[36] covariance             # uncertainty matrix

# Velocity with uncertainty
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear     # linear velocity (x, y, z)
    geometry_msgs/Vector3 angular    # angular velocity (x, y, z)
  float64[36] covariance            # velocity uncertainty
```

### TurtleBot3 Sensor Configuration

- **Camera**: RGB camera for visual perception
- **LiDAR**: 360° laser scanner for obstacle detection and mapping
- **IMU**: Inertial measurement unit for orientation and acceleration
- **Wheel Encoders**: Track wheel rotation for odometry calculation

---

## Enhanced Lab Features

This lab includes a **custom TurtleBot3 monitoring package** (`turtlebot3_lab05`) with advanced features:

### Custom Monitoring Nodes

1. **Real-time Odometry Monitor** (`odometry_monitor`)
   - Live position and velocity tracking
   - Commanded vs. actual velocity comparison
   - Distance calculations and obstacle detection
   - TF transform status monitoring

2. **Sensor Data Logger** (`sensor_logger`)
   - Automated data collection from all sensors
   - JSON export for analysis
   - Statistical summaries of sensor performance

### Complete Lab Launch

**Quick Start - All-in-One Launch:**
```bash
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=30
source install/setup.bash
ros2 launch turtlebot3_lab05 lab05_complete.launch.py
```

This launches:
- Gazebo simulation with TurtleBot3
- RViz with pre-configured sensor displays
- Real-time odometry monitoring
- Sensor data logging
- All required ROS-Gazebo bridges

### Control Methods

If keyboard teleop encounters terminal issues, use direct command publishing:

1. **You can use teleopkey node from lap04 but you need to modify the message type**
2. **Use this ready to use node**
```
  export TURTLEBOT3_MODEL=waffle_pi
  export ROS_DOMAIN_ID=30
  source install/setup.bash
  ros2 run turtlebot3_teleop teleop_keyboard
```
3. **Manual Control via Topic Publishing:**
```bash
# Move forward and turn right
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
  twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}'

# Stop the robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
  twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}'
```

**Note**: TurtleBot3 in this lab uses `TwistStamped` messages on `/cmd_vel` topic, which includes header information for better timestamp tracking.



---

## RViz Display Configuration

### Essential Displays for This Lab

1. **Robot Model**
   - Shows TurtleBot3 3D model
   - Topic: Robot description

2. **LaserScan**
   - Topic: `/scan`
   - Style: Points or Spheres
   - Size: 0.1m
   - Color: By intensity or fixed

3. **Image**
   - Topic: `/camera/image_raw`
   - Transport: compressed (for performance)

4. **Odometry**
   - Topic: `/odom`
   - Arrow Length: 0.3m
   - Show Trail: Yes (last 100 poses)

5. **TF**
   - Shows coordinate frame relationships
   - Enable frames: base_link, odom, map

### Saving RViz Configuration

Save your RViz setup for future use:
- File → Save Config As → `turtlebot3_lab05.rviz`

---

## Topic Analysis Commands

```bash
# List all active topics
ros2 topic list

# Get topic information
ros2 topic info /scan
ros2 topic info /odom
ros2 topic info /camera/image_raw

# Check message frequency
ros2 topic hz /scan
ros2 topic hz /odom

# View message structure
ros2 interface show sensor_msgs/msg/LaserScan
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show sensor_msgs/msg/Image
```

---

## Understanding TurtleBot3 TF Tree

The Transform (TF) tree shows the spatial relationships between coordinate frames:

```
map → odom → base_footprint → base_link → sensors (camera_link, base_scan)
```

- **map**: Global reference frame
- **odom**: Odometry frame (drifts over time)
- **base_link**: Robot's main body frame
- **base_scan**: LiDAR sensor frame
- **camera_link**: Camera sensor frame

View the TF tree:
```bash
ros2 run tf2_tools view_frames
```

---

## Troubleshooting

### Common Issues

1. **TurtleBot3 Model Not Set**
   ```bash
   export TURTLEBOT3_MODEL=waffle_pi
   ```

2. **Missing TurtleBot3 Gazebo Package**
   - Error: `Package 'turtlebot3_gazebo' not found`
   - Solution: Ensure `turtlebot3_simulations` repository is cloned and built:
   ```bash
   cd src/
   git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
   cd .. && colcon build --symlink-install
   ```

3. **RViz Shows No Robot Model**
   - Check that robot_state_publisher is running
   - Verify URDF is being published

4. **No Camera Image in RViz**
   - Check camera plugin in Gazebo
   - Verify topic name: `/camera/image_raw`

5. **LiDAR Data Not Visible**
   - Ensure Fixed Frame is set correctly
   - Check LaserScan topic: `/scan`

6. **Teleop Keyboard Issues**
   ```bash
   # Alternative: Direct velocity command publishing
   ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
   '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ""},
     twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}'
   ```

7. **Robot Not Moving Despite Commands**
   - **Check Gazebo simulation state**: Ensure simulation is not paused
   - **Verify command reception**: Use odometry monitor to confirm velocity commands are received
   - **Check topic connection**: Verify `/cmd_vel` topic has publishers and subscribers
   - **Gazebo physics**: Restart Gazebo if physics engine becomes unresponsive

### Useful Debugging Commands

```bash
# Check running nodes
ros2 node list

# Verify transforms
ros2 run tf2_ros tf2_echo map base_link

# Monitor system performance
ros2 node info /gazebo

# Check topic connections
ros2 topic info /cmd_vel
ros2 topic info /odom

# Verify message flow
ros2 topic echo /cmd_vel --once
ros2 topic echo /odom --once

# Test workspace build
./test_lab05.sh
```

### Performance Optimization

- **Gazebo Performance**: If simulation runs slowly, reduce camera quality or disable unused plugins
- **RViz Performance**: Limit point cloud size and reduce update rates for better visualization performance
- **Network Issues**: Ensure ROS_DOMAIN_ID is set consistently across all terminals

---

## Expected Learning Outcomes

After completing this lab, you should understand:

1. **TurtleBot3 Architecture**: Robot model, sensors, and coordinate frames
2. **RViz Visualization**: How to configure and use RViz for sensor monitoring
3. **Odometry Concepts**: Position tracking, uncertainty, and limitations
4. **Sensor Integration**: How camera and LiDAR data complement each other
5. **TF Relationships**: Coordinate frame transformations in ROS 2

---

## Next Steps

This lab prepares you for:
- **Lab 06**: SLAM (Simultaneous Localization and Mapping)
- **Lab 07**: Autonomous Navigation with Nav2
- Advanced sensor fusion and localization techniques

---

## References

- [TurtleBot3 Official Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS 2 RViz User Guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide.html)
- [Understanding TF2](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Nav2 Odometry Documentation](https://navigation.ros.org/concepts/index.html#odometry)
