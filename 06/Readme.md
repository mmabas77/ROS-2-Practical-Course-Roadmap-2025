# TurtleBot3 Lab 06 - SLAM (Simultaneous Localization and Mapping)

This lab introduces students to SLAM concepts using TurtleBot3 simulation with the SLAM Toolbox package. Students will learn how robots can simultaneously build maps of unknown environments while tracking their own location within those maps.

## Learning Objectives

By completing this lab, students will:
- Understand the fundamentals of SLAM algorithms
- Experience real-time mapping with laser scanner data
- Learn about coordinate frame transformations in robotics
- Practice teleop control for robot exploration
- Visualize mapping data in RViz

## Prerequisites

- Completion of Lab 05 (TurtleBot3 simulation basics)
- Understanding of ROS 2 topics, nodes, and coordinate frames
- SLAM Toolbox package installed (`sudo apt install ros-jazzy-slam-toolbox`)

## What This Lab Includes

### Core Components
- **Gazebo Simulation**: TurtleBot3 Waffle Pi robot in TurtleBot3 World
- **SLAM Toolbox**: Real-time mapping using sync SLAM for reliability
- **RViz Visualization**: Interactive map display and robot tracking
- **Simplified Lifecycle**: Direct SLAM node startup without complex lifecycle management

### Key Files
- `launch/lab06.launch.py`: Main launch file with SLAM integration
- `config/slam_config.yaml`: Optimized SLAM Toolbox configuration
- `config/turtlebot3_lab06_slam.rviz`: Custom RViz setup for SLAM visualization
- `turtlebot3_lab06/map_saver.py`: Educational map saving utility

## Quick Start

### 1. Set Environment
```bash
export TURTLEBOT3_MODEL=waffle_pi
```

### 2. Launch the Lab
```bash
cd /path/to/your/ros_ws
source install/setup.bash
ros2 launch turtlebot3_lab06 lab06.launch.py
```

### 3. Control the Robot
In a new terminal:
```bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```

### 4. Activate SLAM (Required!)
**IMPORTANT**: SLAM Toolbox needs manual activation after launch:

In a new terminal:
```bash
source install/setup.bash
# Configure SLAM Toolbox
ros2 lifecycle set /slam_toolbox configure
# Activate SLAM Toolbox
ros2 lifecycle set /slam_toolbox activate
```

**Check SLAM Status:**
```bash
# Verify SLAM is active
ros2 lifecycle get /slam_toolbox
# Should show: active [3]

# Verify map is being published
ros2 topic echo /map --once
```

### 5. Explore and Map
Use keyboard controls to drive the robot around:
- **w**: forward
- **s**: backward
- **a**: turn left
- **d**: turn right
- **x**: stop
- **q/z**: increase/decrease linear speed
- **e/c**: increase/decrease angular speed

## What You Should See

### In Gazebo
- TurtleBot3 robot in the simulated world
- Robot responding to teleop commands
- Laser scanner visualization (red lines)

### In RViz
- **Map Display**: Real-time map being built (gray=unknown, white=free space, black=obstacles)
- **Robot Model**: 3D representation of TurtleBot3
- **Laser Scan**: Red/colored points showing current sensor readings
- **Robot Path**: Green line showing the robot's trajectory
- **TF Frames**: Coordinate frame relationships (map → odom → base_link)

## SLAM Concepts Demonstrated

### 1. Real-Time Mapping
- Map updates every 0.1 seconds as robot moves
- Laser scanner data converted to occupancy grid
- Unknown areas gradually filled in during exploration

### 2. Localization
- Robot position tracked within the growing map
- Odometry combined with scan matching for accuracy
- Loop closure detection when revisiting areas

### 3. Coordinate Frames
- **map**: Global coordinate frame for the map
- **odom**: Odometry frame (starts at robot's initial position)
- **base_link**: Robot's local coordinate frame

### 4. Sensor Integration
- 2D LIDAR scanner provides distance measurements
- IMU data for orientation tracking
- Wheel encoders for odometry estimation

## Technical Details

### SLAM Algorithm
- Uses **SLAM Toolbox** with Ceres Solver optimization
- Implements graph-based SLAM with pose graph optimization
- Configured for educational use with reliable sync SLAM approach

## Sync vs Async SLAM Comparison

This lab uses **Sync SLAM Toolbox** for educational reliability. Here's the comparison:

### Sync SLAM Toolbox (`sync_slam_toolbox_node`)
✅ **Advantages:**
- **Deterministic**: Same inputs always produce same outputs
- **Easier to debug**: Sequential processing, predictable behavior
- **Educational friendly**: Simpler to understand and troubleshoot
- **Stable**: Less prone to timing-related issues
- **Real-time safe**: Guarantees processing within time constraints

⚠️ **Disadvantages:**
- **Slower processing**: Can't utilize multiple CPU cores for mapping
- **Higher latency**: Waits for each step to complete before moving to next
- **Less efficient**: May skip scans if processing takes too long

### Async SLAM Toolbox (`async_slam_toolbox_node`)
✅ **Advantages:**
- **Higher performance**: Can utilize multiple CPU cores
- **Lower latency**: Processes scans in parallel threads
- **More efficient**: Better resource utilization
- **Scalable**: Handles high-frequency sensor data better

⚠️ **Disadvantages:**
- **Non-deterministic**: Same inputs may produce slightly different outputs
- **Complex debugging**: Multi-threaded processing harder to troubleshoot
- **Timing sensitive**: Requires careful lifecycle management
- **Resource intensive**: Uses more CPU and memory

### When to Use Each:

| **Use Sync SLAM When:** | **Use Async SLAM When:** |
|-------------------------|--------------------------|
| Learning and education | Production robotics systems |
| Debugging SLAM issues | High-frequency sensor data (>10Hz) |
| Limited computational resources | Multi-core systems available |
| Deterministic results needed | Maximum performance required |
| Simple robot platforms | Complex autonomous systems |

### Lifecycle Management Requirement

Both sync and async SLAM Toolbox require **lifecycle management**:

```bash
# Required for BOTH sync and async SLAM
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

**Why lifecycle management?**
- SLAM Toolbox nodes are **lifecycle nodes** (managed nodes)
- They start in "unconfigured" state for safety
- Manual activation ensures controlled startup sequence
- Prevents SLAM from starting before all sensors are ready

### Key Parameters
- **Resolution**: 0.05m per pixel (5cm grid) for detailed mapping
- **Update Rate**: 50Hz transforms, 2Hz map updates for smooth operation
- **Scan Range**: Up to 12 meters (TurtleBot3 Waffle Pi LDS range)
- **Loop Closure**: Enabled for map consistency
- **SLAM Type**: Sync SLAM Toolbox for educational reliability

### RViz Configuration
- Fixed Frame: `odom` (prevents "Fixed Frame does not exist" errors)
- Map topic: `/map` (published by SLAM Toolbox)
- Scan topic: `/scan` (laser scanner data)
- Path topic: `/path` (robot trajectory)

## Educational Activities

### Beginner Activities
1. **Basic Exploration**: Drive robot around and watch map develop
2. **Frame Understanding**: Observe TF relationships in RViz
3. **Sensor Analysis**: Watch laser scan data in different environments

### Intermediate Activities
1. **Systematic Mapping**: Plan efficient exploration patterns
2. **Loop Closure**: Return to starting position and observe map correction
3. **Parameter Tuning**: Modify `slam_config.yaml` and observe effects

### Advanced Activities
1. **Map Analysis**: Save maps and analyze quality metrics
2. **Algorithm Comparison**: Compare with different SLAM approaches
3. **Custom Environments**: Test in different Gazebo worlds

## Troubleshooting

### "No map received" in RViz
- **Most Common Issue**: SLAM Toolbox not activated
- **Solution**: Manually activate SLAM:
  ```bash
  ros2 lifecycle set /slam_toolbox configure
  ros2 lifecycle set /slam_toolbox activate
  ```
- **Check SLAM state**: `ros2 lifecycle get /slam_toolbox` should show `active [3]`
- **Verify map topic**: `ros2 topic echo /map --once` should show map data

### SLAM node not starting
- **Check if node exists**: `ros2 node list | grep slam_toolbox`
- **Check lifecycle state**: `ros2 lifecycle get /slam_toolbox`
- **Expected states**: unconfigured [1] → inactive [2] → active [3]
- **If stuck in unconfigured**: Run configure and activate commands above

### Robot not responding to teleop
- Check TurtleBot3 model: `echo $TURTLEBOT3_MODEL` should show "waffle_pi"
- Ensure teleop is sourced: `source install/setup.bash` in teleop terminal
- Verify cmd_vel topic: `ros2 topic list | grep cmd_vel`

### Map not updating
- Move the robot - SLAM needs motion to build maps
- Check `/scan` topic: `ros2 topic echo /scan --once`
- Verify laser scanner is working in Gazebo (red laser lines visible)

### RViz frame errors
- RViz Fixed Frame is set to `odom` (not `map`) to prevent startup errors
- TF tree: `ros2 run tf2_tools view_frames` to debug frame issues
- Ensure TurtleBot3 model is set correctly for proper frame publishing

### Launch file errors
- **Fixed**: Removed complex lifecycle management that was causing failures
- **Fixed**: Added automatic TURTLEBOT3_MODEL environment variable setting
- If package not found, ensure workspace is built: `colcon build --packages-select turtlebot3_lab06`

## Extension Ideas

### For Students
- Compare mapping quality in different environments
- Implement autonomous exploration algorithms
- Study the effect of robot speed on map quality
- Create custom worlds for mapping challenges

### For Instructors
- Add navigation stack (Nav2) for autonomous navigation
- Integrate multiple robots for multi-robot SLAM
- Add semantic mapping with camera data
- Implement map merging from different exploration sessions

## Files Structure
```
turtlebot3_lab06/
├── README.md                          # This file
├── QUICKSTART.md                      # Quick reference
├── package.xml                        # Package dependencies
├── setup.py                          # Package setup
├── launch/
│   └── lab06.launch.py               # Main SLAM launch file
├── config/
│   ├── slam_config.yaml              # SLAM Toolbox configuration
│   └── turtlebot3_lab06_slam.rviz    # RViz setup file
└── turtlebot3_lab06/
    ├── __init__.py
    └── map_saver.py                  # Educational map saving utility
```

## Next Steps

After completing this lab, students should be ready for:
- **Navigation**: Using maps for autonomous path planning
- **Advanced SLAM**: 3D mapping, semantic SLAM, or multi-robot scenarios
- **Real Robot Deployment**: Applying SLAM concepts to physical robots
- **Research Projects**: Exploring cutting-edge SLAM algorithms

## Support and Resources

- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [ROS 2 Navigation Stack](https://navigation.ros.org/)
- Course discussion forum for questions and collaboration

---

**Remember**: SLAM is a fundamental robotics capability that enables autonomous systems to understand and navigate their environment. This lab provides hands-on experience with the core concepts that power everything from vacuum cleaners to autonomous vehicles.