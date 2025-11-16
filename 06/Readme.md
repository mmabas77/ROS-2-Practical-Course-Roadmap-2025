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
- **Gazebo Simulation**: TurtleBot3 Burger robot in TurtleBot3 World
- **SLAM Toolbox**: Real-time mapping using laser scanner data
- **RViz Visualization**: Interactive map display and robot tracking
- **Automatic Lifecycle Management**: SLAM node activation handled automatically

### Key Files
- `launch/lab06.launch.py`: Main launch file with SLAM integration
- `config/slam_config.yaml`: Optimized SLAM Toolbox configuration
- `config/turtlebot3_lab06_slam.rviz`: Custom RViz setup for SLAM visualization
- `turtlebot3_lab06/map_saver.py`: Educational map saving utility

## Quick Start

### 1. Set Environment
```bash
export TURTLEBOT3_MODEL=burger
```

### 2. Launch the Lab
```bash
ros2 launch turtlebot3_lab06 lab06.launch.py
```

### 3. Control the Robot
In a new terminal:
```bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

### 4. Explore and Map
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
- Configured for educational use with aggressive mapping parameters

### Key Parameters
- **Resolution**: 0.05m per pixel (5cm grid)
- **Update Rate**: 10Hz for responsive mapping
- **Scan Range**: Up to 12 meters
- **Loop Closure**: Enabled for map consistency

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
- **Fixed**: SLAM Toolbox lifecycle node is automatically activated
- If issue persists, manually run: `ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate`

### Robot not responding to teleop
- Check TurtleBot3 model: `echo $TURTLEBOT3_MODEL` should show "burger"
- Ensure teleop is sourced: `source install/setup.bash` in teleop terminal

### Map not updating
- Move the robot - SLAM needs motion to build maps
- Check `/scan` topic: `ros2 topic echo /scan --once`
- Verify laser scanner is working in Gazebo

### RViz frame errors
- RViz Fixed Frame is set to `odom` (not `map`) to prevent startup errors
- TF tree: `ros2 run tf2_tools view_frames.py` to debug frame issues

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