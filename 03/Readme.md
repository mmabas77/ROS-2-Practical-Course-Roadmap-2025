# Lab: Simulating a Robot Using Gazebo Jetty and ROS 2 Jazzy

**Course:** Robotics Lab  
**Department:** Computer Science, FCIS Mansoura University  
**Semester:** Fall 2025  
**ROS Version:** ROS 2 Jazzy Jalisco  
**Gazebo Version:** Gazebo Jetty (gz-sim)

---

## Table of Contents
1. [Learning Objectives](#learning-objectives)
2. [Prerequisites](#prerequisites)
3. [Important: Gazebo Jetty vs Classic](#important-gazebo-jetty-vs-classic)
4. [Real-World Applications](#real-world-applications)
5. [Part 1: Installation and Setup](#part-1-installation-and-setup)
6. [Part 2: Understanding Gazebo Jetty](#part-2-understanding-gazebo-jetty)
7. [Part 3: Building the Warehouse Robot](#part-3-building-the-warehouse-robot)
8. [Part 4: Building a Warehouse World](#part-4-building-a-warehouse-world)
9. [Part 5: Integrating ROS 2 with Gazebo](#part-5-integrating-ros-2-with-gazebo)
10. [Part 6: Autonomous Wall Following](#part-6-autonomous-wall-following)
11. [Troubleshooting](#troubleshooting)
12. [Submission Requirements](#submission-requirements)

---

## Learning Objectives

By the end of this lab, you will be able to:
- Understand the differences between Gazebo Classic and Gazebo Jetty (gz-sim)
- Create robot models using SDF format for Gazebo Jetty
- Use gz-sim plugins for robot control and sensors
- Integrate ROS 2 Jazzy with Gazebo Jetty using `ros_gz` packages
- Implement autonomous navigation using sensor feedback
- Bridge Gazebo topics to ROS 2 topics
- Create ROS 2 launch files for simulation environments

---

## Prerequisites

- **Ubuntu 24.04 LTS** (Noble Numbat)
- **ROS 2 Jazzy Jalisco** installed
- **Gazebo Jetty** (not Classic Gazebo!)
- Basic understanding of:
  - Linux terminal commands
  - Python programming
  - ROS 2 concepts (nodes, topics, publishers, subscribers)
  - SDF/XML file formats

### Verify Your Installation

```bash
# Check ROS 2 installation
ros2 doctor
# Expected: ros2 doctor version [version number]

# Check Gazebo Jetty installation (NOT classic gazebo!)
gz sim --version
# Should show Gazebo Sim version (not Gazebo Classic)

```

**Note:** If `gz sim --version` doesn't work, you may need to install Gazebo Jetty (see Part 1).

---

## Important: Gazebo Jetty vs Classic

**CRITICAL DIFFERENCES:**

| Feature | Gazebo Classic | Gazebo Jetty (New) |
|---------|----------------|-------------------|
| **Command** | `gazebo` | `gz sim` |
| **Package** | `gazebo_ros_pkgs` | `ros_gz` |
| **Plugin Names** | `libgazebo_ros_*.so` | `gz-sim-*-system` |
| **Model Path** | `GAZEBO_MODEL_PATH` | `GZ_SIM_RESOURCE_PATH` |
| **DiffDrive Plugin** | `libgazebo_ros_diff_drive.so` | `gz-sim-diff-drive-system` |
| **Launch** | `gazebo world.sdf` | `gz sim world.sdf` |

**This lab uses Gazebo Jetty (the new version)!** If you're following old tutorials for Gazebo Classic, they won't work!

---

## Real-World Applications

**Mobile warehouse robots** are extensively used in modern logistics:

- **Amazon Fulfillment Centers**: Autonomous robots transport shelves
- **Manufacturing**: Material handling between workstations
- **Healthcare**: Medication and supply delivery
- **Retail**: Inventory management and stock tracking

**Why Simulation?**
- **Cost-effective**: Test without risking physical hardware
- **Safe**: Experiment with dangerous scenarios
- **Rapid iteration**: Test multiple designs quickly
- **Scalable**: Test with many robots simultaneously

---

## Part 1: Installation and Setup

### Step 1.1: Install Gazebo Jetty

If not already installed:

```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-prerelease $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-prerelease.list > /dev/null
sudo apt-get update
sudo apt-get install gz-jetty
```

**Verify installation:**

```bash
gz sim --version
```

Should show Gazebo Sim version information.

---

### Step 1.2: Install ROS 2 - Gazebo Integration Packages

```bash
# Install ros_gz packages (NOT gazebo_ros_pkgs!)
sudo apt install ros-jazzy-ros-gz
```

This installs:
- `ros-jazzy-ros-gz-bridge`: Bridges Gazebo and ROS topics
- `ros-jazzy-ros-gz-sim`: Launch utilities for Gazebo
- `ros-jazzy-ros-gz-image`: Image bridging

---

### Step 1.3: Set Up Environment

Add to your `~/.bashrc`:

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Set Gazebo resource paths
export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models:$GZ_SIM_RESOURCE_PATH
```

Then reload:

```bash
source ~/.bashrc
```

---

## Part 2: Understanding Gazebo Jetty

### Step 2.1: Launch Gazebo Jetty

```bash
gz sim shapes.sdf
```

**What you should see:**
- A 3D window with various shapes
- Play/pause controls at bottom
- Plugin menu (vertical ellipsis) at top right

**Key Commands:**
- `gz sim`: Launch with GUI
- `gz sim -s`: Server only (headless)
- `gz sim -g`: GUI only
- `gz sim -v 4`: Verbose output

---

### Step 2.2: Explore the Interface

**Important Plugins** (accessible via dropdown menu):
- **Entity Tree**: View all objects in the world
- **3D View**: Main visualization
- **World Control**: Play/pause simulation
- **Transform Control**: Move and rotate objects

**To add plugins:**
1. Click the vertical ellipsis (⋮) in top right
2. Select a plugin from the list
3. Plugin appears in the interface

---

### Step 2.3: Test with a Simple World

Create a test file `~/test_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="test_world">
    
    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Physics System Plugin -->
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    
    <!-- Scene Broadcaster Plugin -->
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    
    <!-- Ground Plane -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>
    
    <!-- Sun -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>
    
  </world>
</sdf>
```

**Launch it:**

```bash
gz sim ~/test_world.sdf
```

---

## Part 3: Building the Warehouse Robot

### Step 3.1: Create Model Directory

```bash
mkdir -p ~/.gz/models/mobile_warehouse_robot
cd ~/.gz/models/mobile_warehouse_robot
```

**Note:** We use `~/.gz/models` (NOT `~/.gazebo/models`)!

---

### Step 3.2: Create model.config

```bash
nano model.config
```

Add:

```xml
<?xml version="1.0"?>
<model>
  <name>Mobile Warehouse Robot</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  
  <author>
    <name>Your Name</name>
    <email>your.email@student.mans.edu.eg</email>
  </author>
  
  <description>
    A differential drive mobile warehouse robot for Gazebo Jetty.
    Features two drive wheels, one caster, and a laser range finder.
  </description>
</model>
```

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

---

### Step 3.3: Create model.sdf

This is the main robot description:

```bash
nano model.sdf
```

Add the complete robot model:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="mobile_warehouse_robot">
    
    <!-- Base Link (Main Body) -->
    <link name="base_link">
      <inertial>
        <pose>0 0 0.12 0 0 0</pose>
        <mass>15.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.5</iyy>
          <iyz>0</iyz>
          <izz>0.3</izz>
        </inertia>
      </inertial>
      
      <!-- Visual -->
      <visual name="base_visual">
        <pose>0 0 0.12 0 0 0</pose>
        <geometry>
          <box>
            <size>0.5 0.4 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.2 0.2 0.8 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      
      <!-- Collision -->
      <collision name="base_collision">
        <pose>0 0 0.12 0 0 0</pose>
        <geometry>
          <box>
            <size>0.5 0.4 0.2</size>
          </box>
        </geometry>
      </collision>
    </link>
    
    <!-- Left Wheel -->
    <link name="left_wheel">
      <pose>0.15 0.25 0.08 -1.5707 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      
      <visual name="left_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
      
      <collision name="left_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    
    <!-- Right Wheel -->
    <link name="right_wheel">
      <pose>0.15 -0.25 0.08 -1.5707 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      
      <visual name="right_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
      
      <collision name="right_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    
    <!-- Caster Wheel (Back Support) -->
    <link name="caster">
      <pose>-0.2 0 0.04 0 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      
      <visual name="caster_visual">
        <geometry>
          <sphere>
            <radius>0.04</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
      
      <collision name="caster_collision">
        <geometry>
          <sphere>
            <radius>0.04</radius>
          </sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.01</mu>
              <mu2>0.01</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    
    <!-- Laser Sensor Link -->
    <link name="lidar_link">
      <pose>0.25 0 0.25 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      
      <visual name="lidar_visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.07</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
      
      <collision name="lidar_collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.07</length>
          </cylinder>
        </geometry>
      </collision>
      
      <!-- GPU Lidar Sensor -->
      <sensor name="gpu_lidar" type="gpu_lidar">
        <pose>0 0 0 0 0 0</pose>
        <topic>lidar</topic>
        <update_rate>10</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <samples>640</samples>
              <resolution>1</resolution>
              <min_angle>-1.57</min_angle>
              <max_angle>1.57</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.08</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
          </range>
        </lidar>
        <always_on>1</always_on>
        <visualize>true</visualize>
      </sensor>
    </link>
    
    <!-- JOINTS -->
    
    <!-- Left Wheel Joint -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+12</lower>
          <upper>1e+12</upper>
        </limit>
      </axis>
    </joint>
    
    <!-- Right Wheel Joint -->
    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+12</lower>
          <upper>1e+12</upper>
        </limit>
      </axis>
    </joint>
    
    <!-- Caster Joint -->
    <joint name="caster_joint" type="ball">
      <parent>base_link</parent>
      <child>caster</child>
    </joint>
    
    <!-- Lidar Joint -->
    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
    </joint>
    
    <!-- PLUGINS -->
    
    <!-- Differential Drive Plugin -->
    <plugin
      filename="gz-sim-diff-drive-system"
      name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.5</wheel_separation>
      <wheel_radius>0.08</wheel_radius>
      <odom_publish_frequency>50</odom_publish_frequency>
      <topic>/model/warehouse_robot/cmd_vel</topic>
      <odom_topic>/model/warehouse_robot/odometry</odom_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
    </plugin>
    
  </model>
</sdf>
```

**Key Points:**
- Uses **gz-sim-diff-drive-system** (NOT libgazebo_ros_diff_drive.so!)
- SDF version 1.8 (modern version)
- GPU lidar sensor (more performant)
- All units in meters and radians
- **IMPORTANT:** Uses **absolute topic paths** (`/model/warehouse_robot/cmd_vel`) to ensure proper bridge communication

Save and exit.

**Note:** The absolute topic paths are crucial! If you use relative paths like `<topic>cmd_vel</topic>`, the robot may not respond to ROS commands due to topic name mismatches with the bridge. See Lab 04 for detailed explanation.

---

### Step 3.4: Test the Robot

```bash
gz sim model.sdf
```

**If the robot doesn't appear:**
- Check that you created files in `~/.gz/models/mobile_warehouse_robot/`
- Verify both `model.config` and `model.sdf` exist
- Try restarting Gazebo

**Common Issues:**
- Robot falls through ground: Add a ground plane model
- Robot is invisible: Check SDF syntax errors

Close Gazebo (`Ctrl+C` in terminal).

---

## Part 4: Building a Warehouse World

### Step 4.1: Create World Directory

```bash
mkdir -p ~/.gz/models/small_warehouse
cd ~/.gz/models/small_warehouse
```

---

### Step 4.2: Create model.config

```bash
nano model.config
```

```xml
<?xml version="1.0"?>
<model>
  <name>Small Warehouse</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  
  <author>
    <name>Your Name</name>
    <email>your.email@student.mans.edu.eg</email>
  </author>
  
  <description>
    A small warehouse environment with walls for testing robot navigation.
  </description>
</model>
```

---

### Step 4.3: Create model.sdf

```bash
nano model.sdf
```

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="warehouse_world">
    
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Warehouse using model:// -->
    <include>
      <name>warehouse</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>model://small_warehouse</uri>
    </include>
    
    <!-- Robot using model:// -->
    <include>
      <name>warehouse_robot</name>
      <pose>-3 -3 0.1 0 0 0</pose>
      <uri>model://mobile_warehouse_robot</uri>
    </include>
    
  </world>
</sdf>

```

---

### Step 4.4: Create Complete World File

Now create a world file that includes everything:

```bash
mkdir -p ./ros_ws/worlds
nano ./ros_ws/worlds/warehouse.sdf
```

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="warehouse_world">
    
    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Required Plugins -->
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    
    <!-- GUI -->
    <gui fullscreen="0">
      <!-- 3D View -->
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>-8 -8 8 0 0.5 0.78</camera_pose>
      </plugin>
      
      <!-- World Control -->
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>false</start_paused>
      </plugin>
      
      <!-- World Stats -->
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>
      
      <!-- Entity Tree -->
      <plugin filename="EntityTree" name="Entity tree">
      </plugin>
    </gui>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground Plane -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>
    
    <!-- Warehouse -->
    <include>
      <name>warehouse</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>model://small_warehouse</uri>
    </include>
    
    <!-- Robot -->
    <include>
      <name>warehouse_robot</name>
      <pose>-4 -4 0.1 0 0 0</pose>
      <uri>model://mobile_warehouse_robot</uri>
    </include>
    
  </world>
</sdf>
```

**Key Points:**
- Uses gz-sim plugins (NOT gazebo_ros plugins!)
- Includes all necessary system plugins
- GUI configuration for better user experience
- Models referenced via `model://` URIs

---

### Step 4.5: Test the World

```bash
gz sim ./ros_ws/worlds/warehouse.sdf
```

**What you should see:**
- Warehouse with walls
- Robot in the starting position
- 3D view, controls, and entity tree

**If models don't load:**
- Check `GZ_SIM_RESOURCE_PATH` includes `~/.gz/models`
- Verify model directories exist
- Check SDF syntax errors

Close Gazebo.

---

## Part 5: Integrating ROS 2 with Gazebo

Now we'll bridge Gazebo topics to ROS 2 topics so we can control the robot from ROS.

### Step 5.1: Understand ros_gz_bridge

The `ros_gz_bridge` translates messages between:
- **Gazebo Transport** (gz.msgs.*)
- **ROS 2** (geometry_msgs, sensor_msgs, etc.)

**Bridge configuration:**
- Can be defined in YAML files
- Or created programmatically

---


### Step 5.2: Create ROS 2 Workspace

- We do have ros_ws

---

### Step 5.3: Create Launcher Package

```bash
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws
ros2 pkg create --build-type ament_python warehouse_simulation
```

---

### Step 5.4: Create Bridge Configuration

**Note:** For ament_python packages, `launch/` and `config/` folders go at the package root, NOT in a `src` subfolder.

```bash
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws/src/warehouse_simulation
mkdir -p config launch
nano config/bridge.yaml
```

Add:

```yaml
# Bridge configuration for warehouse robot

# Command velocity (ROS -> Gazebo)
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/warehouse_robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

# Odometry (Gazebo -> ROS)
- ros_topic_name: "/odom"
  gz_topic_name: "/model/warehouse_robot/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

# Lidar (Gazebo -> ROS)
- ros_topic_name: "/scan"
  gz_topic_name: "/world/warehouse_world/model/warehouse_robot/link/lidar_link/sensor/gpu_lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

# TF (Gazebo -> ROS)
- ros_topic_name: "/tf"
  gz_topic_name: "/model/warehouse_robot/pose"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
```

**This bridges:**
- ROS `/cmd_vel` → Gazebo robot velocity
- Gazebo odometry → ROS `/odom`
- Gazebo lidar → ROS `/scan`
- Gazebo pose → ROS `/tf`

---

### Step 5.5: Create Launch File

```bash
nano launch/warehouse_simulation.launch.py
```

Add:

```python
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_warehouse_sim = get_package_share_directory('warehouse_simulation')
    
    # Paths - use os.path.join() instead of PathJoinSubstitution for node parameters
    world_file = os.path.join(os.path.expanduser('~'), 'warehouse_ws', 'worlds', 'warehouse.sdf')
    bridge_config = os.path.join(pkg_warehouse_sim, 'config', 'bridge.yaml')
    
    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(os.path.expanduser('~'), '.gz', 'models')
    )
    
    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )
    
    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        bridge,
    ])
```

**What this does:**
1. Sets `GZ_SIM_RESOURCE_PATH` for model finding
2. Launches Gazebo with the warehouse world
3. Starts the bridge using our YAML config

---

### Step 5.6: Verify Package Structure

Your package should now look like this:

```
warehouse_simulation/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── warehouse_simulation
├── warehouse_simulation/          # Python module directory
│   └── __init__.py
├── launch/                        # At package root!
│   └── warehouse_simulation.launch.py
└── config/                        # At package root!
    └── bridge.yaml
```

**Important:** `launch/` and `config/` are at the same level as `setup.py`, NOT inside any `src` folder!

---

### Step 5.7: Update Package Files

**Edit package.xml:**

```bash
nano package.xml
```

Add these dependencies inside the `<package>` tag:

```xml
<depend>ros_gz_sim</depend>
<depend>ros_gz_bridge</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>sensor_msgs</depend>
<depend>tf2_msgs</depend>
```

---

**Edit setup.py:**

```bash
nano setup.py
```

Make sure it looks exactly like this:

```python
import os
from glob import glob
from setuptools import setup

package_name = 'warehouse_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@student.mans.edu.eg',
    description='Warehouse robot simulation with Gazebo Jetty',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

**Key point:** The `glob()` functions look for `launch/*.launch.py` and `config/*.yaml` relative to where `setup.py` is located (the package root).

---

### Step 5.8: Build the Package

```bash
cd ~/Desktop/ROS-2-Practical-Course-Roadmap-2025/ros_ws
colcon build --packages-select warehouse_simulation
source install/setup.bash
```

---

### Step 5.9: Verify Installation

Check that files were installed correctly:

```bash
ls -la install/warehouse_simulation/share/warehouse_simulation/launch/
ls -la install/warehouse_simulation/share/warehouse_simulation/config/
```

You should see:
- `warehouse_simulation.launch.py` in the launch directory
- `bridge.yaml` in the config directory

---

### Step 5.10: Launch the Simulation

```bash
ros2 launch warehouse_simulation warehouse_simulation.launch.py
```

**What should happen:**
- Gazebo opens with warehouse and robot
- ROS-Gazebo bridge starts
- Topics are bridged (`/cmd_vel`, `/odom`, `/scan`, `/tf`)


### Step 5.9: Verify ROS Topics

**Open a new terminal:**

```bash
source ./ros_ws/install/setup.bash
ros2 topic list
```

**You should see:**
```
/cmd_vel
/odom
/scan
/tf
...
```


