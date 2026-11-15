# ArUco Treasure Hunt

The assignment involves programming a robot to take part in a treasure hunt.
There are four ArUco markers to reach in order; the robot must stop approximately 50 cm from each marker; the marker’s ID will provide the instruction to continue along the route.

<div align="center">
  <img src="readme_img/ArUco_map.png" width="60%"/>
  <br>
  <p>Figure 1: map showing ArUco's positions</p>
</div>

*(\*) The robot is initially positioned so that it faces marker **1** directly.*


--- 

# Repository structure

There are only two branches to consider for this assignment
* main: implements the simulation in RViz
* lab_setup: implements the code for connecting to the real robot

Both are organised in the same way, with three packages:
* custom_msgs: containing the custom message 
* erl1: the actual project folder, containing the models, configuration files, launch file and nodes
* ros_aruco_opencv: required to use Aruco

---

# Nodes

The programme is divided into three nodes. The following names of the topics used in the README reflect the remapped names (as shown in the various yaml files)

### 1. marker_image (.py)

It is responsible for sensing the environment. It receives images from the `/camera` and detects markers using the `/aruco_detections` topic.

To visually highlight the target with a bounding circle on the debug image published on the `/marker_image` topic and to ensure the correct sequence of the ‘order’ of the markers. It analyses their IDs and, if they match the correct marker, it processes it. To know which target number it needs to follow, it subscribes to the `/target_reached` topic.

It calculates the marker’s apparent dimensions in pixels considering radius $r$, extracts its 3D coordinates (x, y, z) and converting them to the corresponding 2D values by projecting the coordinates onto the image plane:

$$u = f_x \frac{X}{Z} + c_x$$

$$v = f_y \frac{Y}{Z} + c_y$$

where:
- **$(u, v)$**: 2D pixel coordinates of the marker center in the image (`cx`, `cy`)
- **$(X, Y, Z)$**: 3D position of the marker relative to the camera frame (with $Z$ representing depth `d`)
- **$f_x, f_y$**: focal lengths in pixels ($K_{0,0}$ and $K_{1,1}$)
- **$c_x, c_y$**: optical center coordinates in pixels ($K_{0,2}$ and $K_{1,2}$)

Finally, it sends a custom message on the **/marker_info** topic containing the marker’s 2D coordinates (`cx`, `cy`), the depth `d` ($Z$), and its ID.


### 2. velocity (.cpp)

This is the node responsible for calculating the Visual Servoing. 

By subscribing to the `/marker_info` topic, it retrieves the data required to calculate the correct angular and linear velocities needed to centre the marker and reach the desired proximity.

These values are published to the `/velocity_info` topic; once the correct distance has been reached (hence the robot is at least 50 cm away from the marker), the node publishes the marker_id number of the marker reached to the `/target_reached` topic.

#### The formulas used to compute the error are the following:

* To determine the desired marker size

    $$s_d = \frac{f \cdot L}{d}$$

    * **$s_d$**: desired size of the marker’s side in the image (166 px)
    * **$f$**: focal length of the camera (px, derived from the `camera_info` calibration parameters)
    * **$L$**: actual physical dimension of the marker’s side (m)
    * **$d$**: desired distance to maintain from the marker (about 0.5 m)

* To determine the angular velocity
    $$\omega = -k_\omega \cdot (u - c_x)$$

    * **$\omega$**: angular velocity commanded to the robot ($\text{rad/s}$)
    * **$k_\omega$**: proportional gain for angular control
    * **$u$**: horizontal coordinate (px) of the centre of the detected marker
    * **$c_x$**: horizontal coordinate (px) of the centre of the image

* To determine the linear velocity

    $$v = k_v \cdot (s_d - s)$$

    * **$v$**: linear velocity commanded to the robot ($\text{m/s}$)
    * **$k_v$**: proportional gain for linear control
    * **$s_d$**: desired size of the marker’s side (px)
    * **$s$**: current apparent size of the detected marker side (px)

### 3. treasure_finder (.py)

This is the node that coordinates the robot’s actual movement.

It estimates the robot’s actual orientation (yaw) by subscribing to the `/odometry` topic and converting the quaternion $(x, y, z, w)$:

$$\psi = \text{atan2}\left(2(w z + x y), \; 1 - 2(y^2 + z^2)\right)$$

Taking into account the required velocities published on the `/velocity_info` topic, the instruction to follow through the ID published on `/target_reached`, and the currently seen ID from `/marker_info` (to ignore markers already visited), it implements a state machine capable of determining the robot’s movement by publishing the commands on `/cmd_vel`.

Once the marker has been reached, its ID is published on the `/target_reached` topic; it can then extract this ID to execute the correct instruction:

| Marker ID | Command |
| :---: | :--- |
| **1** | Turn **right** by 90°, then go straight ahead |
| **2** | Rotate by **180°**, then go straight ahead |
| **3** | Turn **left** by 90°, then go straight ahead |
| **4** | **Stop**: the hunt is over |

#### State machine
The node cycles between three states (`SERVOING`, `ROTATING`, `STOPPED`). During `SERVOING`, it follows `/velocity_info` commands or drives straight ahead ($0.3\text{ m/s}$) if no new marker is detected. Once a target is reached, it switches to `ROTATING` to complete the maneuver using `/odometry`; after the rotation is complete, it publishes **`id = 0`** on `/target_reached` to reset the state and returns to `SERVOING`.

---

# Requirements
Operating System: Ubuntu (suggested: 24.04)
ROS 2: suggested distribution Jazzy
Standard ROS 2 tools:
- `colcon`
- Gazebo
- Rviz

To install **Gazebo and Rviz**:
```
# install Gazebo Harmonic (for Jazzy)
sudo apt update
sudo apt install ros-jazzy-ros-gz

# install RViz2
sudo apt install ros-jazzy-rviz2
```

# Running the simulation

Clone the repository (selecting the desired branch) inside the `src` folder of your ROS 2 workspace:
```
git clone -b main https://github.com/TZTozz/Experimental.git
```

Move to the root of the workspace, build and source the environmet:
```
colcon build
source install/setup.bash
```

Using the launch file to run the simulation:
```
ros2 launch erl1 spawn_assignment.launch.py 
```