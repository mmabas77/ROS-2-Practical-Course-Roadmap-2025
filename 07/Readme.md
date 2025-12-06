# TurtleBot3 Lab 07 - Autonomous Navigation with Nav2

This lab introduces autonomous navigation using the Nav2 stack. Building on SLAM from Lab 06, students learn to use pre-built maps for robot localization and path planning.

## Learning Objectives

- Understand the Nav2 navigation stack architecture
- Learn about AMCL (Adaptive Monte Carlo Localization)
- Experience autonomous path planning and execution
- Practice setting navigation goals using RViz

## Prerequisites

- Completion of Lab 06 (SLAM fundamentals)
- Nav2 packages installed:
  ```bash
  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
  ```

## Quick Start - Lab07 Package

### Step 1: Build the Package

```bash
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws
colcon build --packages-select turtlebot3_lab07
source install/setup.bash
```

### Step 2: Launch Navigation

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_lab07 lab07.launch.py
```

This launches Gazebo, Nav2, and RViz in a single command.

- Example with a map saved in ~/maps:
```
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_lab07 lab07.launch.py map:=$HOME/maps/my_map.yaml
```

### Step 3: Set Initial Pose (REQUIRED)

**IMPORTANT**: You MUST set the initial pose before the robot can navigate.
```
  source /opt/ros/jazzy/setup.bash
  ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 
  'map'}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}" --once
```

### Step 4: Send Navigation Goals

1. Click **"2D Goal Pose"** button in RViz toolbar
2. Click and drag on the map where you want the robot to go
3. Watch the robot autonomously plan a path and navigate!

**Alternative**: Set initial pose via command line:
```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}" --once
```

### Optional: Use Your Own Map from Lab 06

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_lab07 lab07.launch.py map:=$HOME/maps/my_map.yaml
```

## Package Structure

```
ros_ws/turtlebot3_lab07/
├── package.xml                 # Package dependencies
├── setup.py                    # Build configuration
├── resource/turtlebot3_lab07   # Package marker
├── launch/lab07.launch.py      # All-in-one launch file
└── turtlebot3_lab07/__init__.py
```

The launch file starts:
- Gazebo simulation (TurtleBot3 World)
- Nav2 stack (AMCL, planners, controllers, costmaps)
- RViz with navigation visualization

## Nav2 Components

| Component | Description |
|-----------|-------------|
| **AMCL** | Estimates robot pose on a known map |
| **Map Server** | Loads and serves the occupancy grid map |
| **Planner Server** | Computes global paths from start to goal |
| **Controller Server** | Executes path with local obstacle avoidance |
| **BT Navigator** | Behavior Tree-based navigation orchestration |

## Key Topics

| Topic | Description |
|-------|-------------|
| `/map` | Static map from map server |
| `/scan` | LIDAR data for localization |
| `/cmd_vel` | Velocity commands to robot |
| `/goal_pose` | Navigation goal from RViz |
| `/plan` | Computed global path |

## Using RViz for Navigation

### Setting Initial Pose

The robot doesn't know where it is on the map initially. You must tell it:

1. Click **"2D Pose Estimate"** button
2. Click on the map where the robot is located
3. Drag to set the orientation (direction robot is facing)
4. Watch the particle cloud (red arrows) converge

### Sending Navigation Goals

1. Click **"2D Goal Pose"** button
2. Click on the map destination
3. Drag to set desired final orientation
4. Watch the robot plan and execute the path

## Troubleshooting

### Gazebo/RViz Frozen or Static

- Ensure you sourced the workspace: `source install/setup.bash`
- Wait at least 30 seconds for full initialization
- Check if simulation clock is advancing: `ros2 topic echo /clock --once`
- Kill any leftover processes: `pkill -f "ros2|gazebo|rviz"` and retry

### Robot Not Localizing

- Ensure initial pose is set correctly (Step 3)
- The robot spawns at (-2, -0.5) in TurtleBot3 World
- Drive slowly to help AMCL converge

### Navigation Fails to Plan

- Check goal is in free space (white area on map)
- Try a closer goal first
- Verify TF: `ros2 run tf2_tools view_frames`

### "Waiting for Transform" Errors

- Wait longer for initialization (~30 seconds)
- Ensure all nodes started: `ros2 node list`

## Useful Commands

```bash
# Check Nav2 nodes are running
ros2 node list | grep -E "amcl|planner|controller|bt_navigator"

# Monitor navigation status
ros2 topic echo /navigate_to_pose/_action/status

# View TF tree
ros2 run tf2_tools view_frames

# Send navigation goal via command line
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.5}, orientation: {w: 1.0}}}}"
```

## Nav2 Architecture

```
                    ┌─────────────────┐
                    │   RViz Goal     │
                    │  (2D Goal Pose) │
                    └────────┬────────┘
                             │
                             ▼
                    ┌─────────────────┐
                    │  BT Navigator   │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
     ┌────────────┐  ┌────────────┐  ┌────────────┐
     │  Planner   │  │ Controller │  │  Behavior  │
     │ (Global)   │  │  (Local)   │  │ (Recovery) │
     └─────┬──────┘  └─────┬──────┘  └────────────┘
           │               │
           ▼               ▼
     ┌────────────┐  ┌────────────┐
     │  Global    │  │   Local    │
     │  Costmap   │  │  Costmap   │
     └────────────┘  └────────────┘
                   │
            ┌──────┴──────┐
            │    AMCL     │
            │ (Localize)  │
            └─────────────┘
```

## Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [TurtleBot3 Navigation Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/)
