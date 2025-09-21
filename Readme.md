# ROS 2 Practical Course Roadmap
## Python-Focused Development with ROS 2 Jazzy Jalisco

## Prerequisites & Setup

### Software Requirements
- **Operating System:** Ubuntu 24.04 LTS (Noble)
- **ROS Distribution:** ROS 2 Jazzy Jalisco
- **Programming Language:** Python 3.12
- **Simulation:** Gazebo (latest version)
- **Computer Vision:** OpenCV 4.x

### Hardware Requirements
- **Minimum RAM:** 8GB
- **Recommended:** 16GB RAM

### Essential Resources

#### Official Documentation
- [ROS 2 Kilted Installation Guide](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)

#### Learning Platforms
- [Automatic Addison ROS 2 Tutorials](https://automaticaddison.com/tutorials/)
- [The Robotics Back-End](https://roboticsbackend.com/category/ros2/)

#### Video Resources
- [Articulated Robotics YouTube Channel](https://www.youtube.com/c/ArticulatedRobotics)
- [The Construct YouTube Channel](https://www.youtube.com/c/TheConstruct)

---

## Course Curriculum

### **Session 1: ROS 2 Foundation & Core Architecture**
**Goals:**
- Install and configure ROS 2 Kilted environment
- Understand ROS 2 architecture and design principles
- Create your first Python node

**Instructor Resources:**
- [Official ROS 2 Kilted Installation Documentation](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)
- [Writing a Minimal ROS 2 Python Node](https://roboticsbackend.com/write-minimal-ros2-python-node/)
- [Video : Install Ubuntu 18.04 virtual machine (Same as 24.04)](https://youtu.be/O5Puwym7K5E?si=0dsnOFHyuJH0DVTR)
- [Video : What is ROS (Official)](https://vimeo.com/639236696?fl=pl&fe=sh)
- [Video : What is ROS (Arabic)](https://youtu.be/T-k9t3QsoT4?si=poekwSINkc2Z1Q87)
- [Video : Create and Set Up a ROS2 Workspace](https://youtu.be/3GbrKQ7G2P0?si=XIqFRde8GX3fLFhV)
- [Video : Create a ROS2 Python Package](https://youtu.be/iBGZ8LEvkCY?si=5bXdAt6zS8j9olTn)
- [Video : Create a ROS2 Node](https://youtu.be/wfCuPQ_6VbI?si=AiKajCo4dUkEC1NQ)


---

### **Session 2: ROS 2 Communication Patterns & Multi-Node Systems**
**Goals:**
- Master publisher-subscriber communication patterns
- Debug multi-node systems using ROS 2 tools

**Instructor Resources:**
- [Writing Service and Client in Python](https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Writing a TF2 Broadcaster](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html)
- [Video : What is a ROS2 Topic?](https://youtu.be/MwEXX6a-TWw?si=rTNI1aeoFoN7GcSR)
- [Video : Write a ROS2 Publisher with Python](https://youtu.be/Yy4OgGwEAj8?si=uacEtEGokvSVYafM)
- [Video : Write a ROS2 Subscriber with Python](https://youtu.be/od3JwOeyEXc?si=89stHMb8_-DHoplw)

---

### **Sessions 3-4: Gazebo Simulation Fundamentals**
**Goals:**
- Install and configure Gazebo simulation environment
- Design and build a wheeled robot URDF from scratch
- Create custom world with realistic obstacles
- Analyze simulation time vs. real-time performance

**Instructor Resources:**
- [Gazebo Robot Simulation Setup](https://gazebosim.org/docs/ionic/tutorials/)
- [Simulating Robots with Gazebo and ROS 2](https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/)
- GUI
- [Video : Exploring Gazebo (Arabic)](https://youtu.be/KUMHqWTr9lc?si=YnIbCP7is9eGv5CB)
- [Video : Gazebo build your world using building editor](https://youtu.be/pM7jIxXXvFM?si=5aNFwk4vbrQRqaMf)
- [Video : Gazebo build your Robot using Model Editor part 1](https://youtu.be/xB8tJ9o4eNg?si=1TgZLW6cIuThp1rI)
- [Video : Gazebo build your Robot using Model Editor part 2](https://youtu.be/qT6Q2syd1M0?si=8mt7AEXkAlP5Hy0E)
- XML
- [Video : Simulating Robots with Gazebo and ROS](https://youtu.be/laWn7_cj434?si=AnXetkouMSYBD1J2)
- [Video : Creating a rough 3D model of our robot with URDF](https://youtu.be/BcjHyhV0kIs?si=CAZnlBHa6--odYt-)

---

### **Session 5: Robot Control & Sensor Integration**
**Goals:**
- Connect ROS 2 control nodes to Gazebo robot model
- Configure TurtleBot3 with camera and LiDAR sensors
- Implement keyboard teleoperation control

**Instructor Resources:**
- [Official Docs](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Video : Getting Started with TurtleBot3 in ROS](https://youtu.be/e-7SlNDh8A8?si=2L79nas9mMkgUDcL)
- OR
- [Video : How to use Cameras in ROS (Add camera to the bot)](https://youtu.be/A3nw2M47K50?si=PA1_Z_GUaTtkPCXh)
- [Video : How to use Lidar in ROS](https://youtu.be/eJZXRncGaGM?si=n-MrKUWUJYSEjMrS)

---

### **Session 6: SLAM Implementation**
**Goals:**
- Configure SLAM Toolbox for mapping
- Build comprehensive environment map
- Save and load map data for navigation

**Instructor Resources:**
- [Video : Easy SLAM with ROS using slam_toolbox (Video)](https://youtu.be/ZaiA3hWaRzE?si=02UikckogA1UmWMv)
- [TurtleBot3 Camera and LiDAR Scan](https://youtu.be/3hswO5bAIK4?si=nP-mHU7tPEW4N1Ea)

---

### **Session 7: Autonomous Navigation with Nav2**
**Goals:**
- Replace manual control with Nav2 autonomous navigation
- Configure costmaps and planners
- Test navigation in complex environments

**Instructor Resources:**
- [Video : Robot Navigation with Nav2 and ROS (Video)](https://youtu.be/jkoGkAd0GYk?si=Y75rRRf4oD62-eJV)

---

### **Session 8: Computer Vision with OpenCV**
**Goals:**
- Master OpenCV fundamentals in Python
- Implement person detection using HOG descriptors
- Process real-time video streams

**Instructor Resources:**
- Color space conversions (RGB, BGR, Grayscale, HSV, L*a*b) - [1:12:53](https://www.youtube.com/watch?v=oXlwWbU8l2o&t=4373s)
- Channel splitting and merging - [1:23:10](https://www.youtube.com/watch?v=oXlwWbU8l2o&t=4990s)
- Image blurring techniques - [1:31:03](https://www.youtube.com/watch?v=oXlwWbU8l2o&t=5463s)
- Bitwise operations - [1:44:27](https://www.youtube.com/watch?v=oXlwWbU8l2o&t=6267s)
- Image masking - [1:53:06](https://www.youtube.com/watch?v=oXlwWbU8l2o&t=6786s)
- Histogram computation - [2:01:43](https://www.youtube.com/watch?v=oXlwWbU8l2o&t=7303s)
- Thresholding and binarization - [2:15:22](https://www.youtube.com/watch?v=oXlwWbU8l2o&t=8122s)
- Advanced edge detection - [2:26:27](https://www.youtube.com/watch?v=oXlwWbU8l2o&t=8787s)

- [HOG Detection with OpenCV (Video)](https://www.youtube.com/watch?v=UQRW4B4_nmU)

- [Camera Calibration using OpenCV](https://learnopencv.com/camera-calibration-using-opencv/)


---


### **Session 9: ROS 2 Computer Vision Integration**
**Goals:**
- Create image subscriber with OpenCV processing pipeline
- Implement real-time video processing node
- Publish detection results as custom ROS messages
- Visualize processed images using rqt_image_view

**Instructor Resources:**
- [TurtleBot3 Object Following (GitHub)](https://github.com/emirhancibir/turtlebot3_object_following)
- [TurtleBot3 Person Following (GitHub)](https://github.com/mmabas77/ros_ws)
- [Video : HOG Detection with OpenCV](https://www.youtube.com/watch?v=UQRW4B4_nmU)
- [Video : Chase a tennis ball with ROS](https://youtu.be/gISSSbYUZag?si=j64B1IBVucElRDiO)

---

### **Session 10: Introduction to Reinforcement Learning**
**Goals:**
- Understand reinforcement learning fundamentals
- Train lunar lander agent using reinforcement learning
- Evaluate and optimize agent performance

**Instructor Resources:**
- [Lunar Lander Theory (Coursera)](https://www.coursera.org/learn/unsupervised-learning-recommenders-reinforcement-learning/lecture/C9BJf/lunar-lander)
- [Lunar Lander Implementation (GitHub)](https://github.com/greyhatguy007/Machine-Learning-Specialization-Coursera/blob/main/C3%20-%20Unsupervised%20Learning%2C%20Recommenders%2C%20Reinforcement%20Learning/week3/C3W3A1/C3_W3_A1_Assignment.ipynb)
- [Video : Intro to Reinforcement Learning for Beginners in Python](https://youtu.be/Uc6qBg7mM2Y?si=I_DrdM6N2FauNrbb)

---

