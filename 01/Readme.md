## How it works ?

You have a Workspace (EX: for the organisation), this workspace have packages such that every pkg is responsibe for a specific task (EX: Robot controller or camera driver) and every pkg contains a set of nodes communicating with eachothers using pub-sub.

## Install colcon (Build tool that uses the build system "ament")

colcon is a command line tool to improve the workflow of building, testing and using multiple software packages. It automates the process, handles the ordering and sets up the environment to use the packages.

```
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install python3-colcon-common-extensions

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```


## Create Your Workspace

```
mkdir -p ros_ws/src
cd ros_ws
colcon build
source install/setup.bash
```

## Create pkg

- for C++ pkg

  ```
  ros2 pkg create my_first_pkg
  ```
- For python pkgs

  ```
  ros2 pkg create my_first_py_pkg --build-type ament_python --dependencies rclpy
  ```
- Now build the WS again

## Create your first node

```
touch ./my_first_py_pkg/my_first_py_pkg/my_first_node.py
chmod +x ./my_first_py_pkg/my_first_py_pkg/my_first_node.py

```

## Write code to your node

- You can refer to this article [Write a Minimal ROS2 Python Node](https://roboticsbackend.com/write-minimal-ros2-python-node/)

## Add the pkg excutable ROS node and run the node

- Add this to the "ros_ws/my_first_py_pkg/setup.py"

```
entry_points={
        'console_scripts': [
            "test_node = my_first_py_pkg.my_first_node:main"
        ],
    },
```

- Now rebild the WS
- We can use symlink install from colcon so that we don't need to rebild the WS every time we change python code

  ```
  colcon build --symlink-install
  ```
- And then run the node with ros command

  ```ros2
  ros2 run my_first_py_pkg test_node
  ```

## Get info about nodes

```
# Show running nodes - topics & pub sub ops
rqt_graph

# List nodes
ros2 node list

# Get info about a node
ros2 node info /my_first_node
```
